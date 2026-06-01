#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "monitor_interfaces/msg/thermal_camera_event.hpp"

#ifdef USE_HIK_SDK
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "HCNetSDK.h"

#ifndef HIK_SDK_LIB_DIR
#define HIK_SDK_LIB_DIR ""
#endif
#ifndef HIK_SDK_COM_DIR
#define HIK_SDK_COM_DIR ""
#endif
#ifndef HIK_SDK_SSL_PATH
#define HIK_SDK_SSL_PATH ""
#endif
#ifndef HIK_SDK_CRYPTO_PATH
#define HIK_SDK_CRYPTO_PATH ""
#endif

class HikAlarmNode;
static HikAlarmNode *g_node_instance = nullptr;

class HikAlarmNode : public rclcpp::Node
{
public:
    HikAlarmNode() : Node("hik_alarm_node")
    {
        device_ip_ = declare_parameter<std::string>("device_ip", "192.168.2.64");
        device_port_ = declare_parameter<int>("device_port", 8000);
        username_ = declare_parameter<std::string>("username", "admin");
        password_ = declare_parameter<std::string>("password", "FHZNfhzn");
        login_mode_ = declare_parameter<int>("login_mode", 0);
        sdk_log_enable_ = declare_parameter<bool>("sdk_log_enable", true);
        sdk_log_dir_ = declare_parameter<std::string>("sdk_log_dir", "/tmp/hik_sdk_log");

        pub_ = create_publisher<monitor_interfaces::msg::ThermalCameraEvent>("/monitor/camera/thermal_event", 10);

        g_node_instance = this;
        worker_ = std::thread(&HikAlarmNode::run, this);
    }

    ~HikAlarmNode() override
    {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
        cleanup();
        g_node_instance = nullptr;
    }

    void on_alarm(LONG lCommand, char *pAlarmInfo, DWORD dwBufLen)
    {
        monitor_interfaces::msg::ThermalCameraEvent msg;
        msg.header.stamp = now();
        msg.header.frame_id = "hik_thermal";
        msg.command = static_cast<int32_t>(lCommand);

        if (lCommand == 0x4012)
        {
            const std::string raw(pAlarmInfo, pAlarmInfo + dwBufLen);
            msg.has_image = false;
            msg.temperature = 0.0f;
            msg.raw_summary = raw.substr(0, std::min<size_t>(raw.size(), 256));
            pub_->publish(msg);
        }
        else if (lCommand == 0x5212)
        {
            msg.has_image = true;
            msg.temperature = 0.0f;
            msg.raw_summary = "image alarm data";
            msg.image_data.assign(reinterpret_cast<uint8_t *>(pAlarmInfo), reinterpret_cast<uint8_t *>(pAlarmInfo) + dwBufLen);
            pub_->publish(msg);
        }
        else if (lCommand == 0x4993)
        {
            msg.has_image = false;
            msg.temperature = 0.0f;
            msg.raw_summary = "thermometry alarm";
            pub_->publish(msg);
        }
    }

private:
    std::string masked_password() const
    {
        if (password_.empty())
            return "<empty>";
        if (password_.size() <= 2)
            return std::string(password_.size(), '*');
        return std::string(password_.size() - 2, '*') + password_.substr(password_.size() - 2);
    }

    static void CALLBACK MsgCallback(LONG lCommand, NET_DVR_ALARMER *pAlarmer, char *pAlarmInfo, DWORD dwBufLen, void *pUser)
    {
        (void)pAlarmer;
        (void)pUser;
        if (g_node_instance != nullptr && pAlarmInfo != nullptr && dwBufLen > 0)
        {
            g_node_instance->on_alarm(lCommand, pAlarmInfo, dwBufLen);
        }
    }

    void run()
    {
        RCLCPP_INFO(
            get_logger(),
            "[HIK] Startup config: ip=%s port=%d user=%s password=%s login_mode=%d sdk_log_enable=%s",
            device_ip_.c_str(), device_port_, username_.c_str(), masked_password().c_str(), login_mode_, sdk_log_enable_ ? "true" : "false");
        RCLCPP_INFO(
            get_logger(),
            "[HIK] SDK path config: lib_dir=%s com_dir=%s ssl=%s crypto=%s",
            HIK_SDK_LIB_DIR, HIK_SDK_COM_DIR, HIK_SDK_SSL_PATH, HIK_SDK_CRYPTO_PATH);

        if (std::strlen(HIK_SDK_LIB_DIR) > 0)
        {
            NET_DVR_LOCAL_SDK_PATH sdk_path{};
            std::snprintf(sdk_path.sPath, sizeof(sdk_path.sPath), "%s", HIK_SDK_LIB_DIR);
            const bool ok = NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SDK_PATH, &sdk_path);
            RCLCPP_INFO(get_logger(), "[HIK] SetSDKInitCfg(SDK_PATH)=%s path=%s", ok ? "true" : "false", sdk_path.sPath);
        }

        auto first_existing = [](const std::vector<std::string> &candidates) -> std::string
        {
            for (const auto &p : candidates)
            {
                if (!p.empty() && std::filesystem::exists(p))
                    return p;
            }
            return "";
        };

        const std::string crypto_path = first_existing({HIK_SDK_CRYPTO_PATH, std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so.3",
                                                        std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so.1.1", std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so"});
        if (!crypto_path.empty())
        {
            const bool ok = NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_LIBEAY_PATH, const_cast<char *>(crypto_path.c_str()));
            RCLCPP_INFO(get_logger(), "[HIK] SetSDKInitCfg(CRYPTO_PATH)=%s path=%s", ok ? "true" : "false", crypto_path.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "[HIK] No usable libcrypto found under SDK lib dir.");
        }

        const std::string ssl_path = first_existing({HIK_SDK_SSL_PATH, std::string(HIK_SDK_LIB_DIR) + "/libssl.so.3",
                                                     std::string(HIK_SDK_LIB_DIR) + "/libssl.so.1.1", std::string(HIK_SDK_LIB_DIR) + "/libssl.so"});
        if (!ssl_path.empty())
        {
            const bool ok = NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SSLEAY_PATH, const_cast<char *>(ssl_path.c_str()));
            RCLCPP_INFO(get_logger(), "[HIK] SetSDKInitCfg(SSL_PATH)=%s path=%s", ok ? "true" : "false", ssl_path.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "[HIK] No usable libssl found under SDK lib dir.");
        }
        if (sdk_log_enable_)
        {
            NET_DVR_SetLogToFile(3, const_cast<char *>(sdk_log_dir_.c_str()), FALSE);
        }

        if (!NET_DVR_Init())
        {
            RCLCPP_ERROR(get_logger(), "[HIK] SDK init failed: %u", NET_DVR_GetLastError());
            rclcpp::shutdown();
            return;
        }

        NET_DVR_SetConnectTime(2000, 1);
        NET_DVR_SetReconnect(10000, true);

        NET_DVR_USER_LOGIN_INFO login_info{};
        NET_DVR_DEVICEINFO_V40 device_info{};
        strncpy(reinterpret_cast<char *>(login_info.sDeviceAddress), device_ip_.c_str(), sizeof(login_info.sDeviceAddress));
        login_info.wPort = static_cast<WORD>(device_port_);
        strncpy(reinterpret_cast<char *>(login_info.sUserName), username_.c_str(), sizeof(login_info.sUserName));
        strncpy(reinterpret_cast<char *>(login_info.sPassword), password_.c_str(), sizeof(login_info.sPassword));
        login_info.byLoginMode = static_cast<BYTE>(login_mode_);
        login_info.byHttps = 0;

        user_id_ = NET_DVR_Login_V40(&login_info, &device_info);
        if (user_id_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "[HIK] Login failed: %u", NET_DVR_GetLastError());
            NET_DVR_Cleanup();
            rclcpp::shutdown();
            return;
        }

        if (!NET_DVR_SetDVRMessageCallBack_V30(MsgCallback, nullptr))
        {
            RCLCPP_ERROR(get_logger(), "[HIK] Set callback failed: %u", NET_DVR_GetLastError());
            cleanup();
            rclcpp::shutdown();
            return;
        }

        NET_DVR_SETUPALARM_PARAM setup_param{};
        setup_param.dwSize = sizeof(NET_DVR_SETUPALARM_PARAM);
        setup_param.byLevel = 0;
        setup_param.byAlarmInfoType = 1;

        alarm_handle_ = NET_DVR_SetupAlarmChan_V41(user_id_, &setup_param);
        if (alarm_handle_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "[HIK] Setup alarm failed: %u", NET_DVR_GetLastError());
            cleanup();
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "[HIK] Alarm subscription started.");
        while (rclcpp::ok() && running_)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void cleanup()
    {
        if (alarm_handle_ >= 0)
        {
            NET_DVR_CloseAlarmChan_V30(alarm_handle_);
            alarm_handle_ = -1;
        }
        if (user_id_ >= 0)
        {
            NET_DVR_Logout_V30(user_id_);
            user_id_ = -1;
        }
        NET_DVR_Cleanup();
    }

    std::string device_ip_;
    int device_port_{};
    std::string username_;
    std::string password_;
    int login_mode_{};
    bool sdk_log_enable_{};
    std::string sdk_log_dir_;

    std::atomic<bool> running_{true};
    std::thread worker_;
    LONG user_id_{-1};
    LONG alarm_handle_{-1};

    rclcpp::Publisher<monitor_interfaces::msg::ThermalCameraEvent>::SharedPtr pub_;
};

#else
class HikAlarmNode : public rclcpp::Node
{
public:
    HikAlarmNode() : Node("hik_alarm_node")
    {
        timer_ = create_wall_timer(std::chrono::seconds(5), [this]()
                                   { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Hik SDK disabled on this architecture. Build/run on ARM64 target to enable camera alarms."); });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikAlarmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
