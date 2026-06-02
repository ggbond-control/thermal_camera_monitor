#include <atomic>
#include <algorithm>
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
static constexpr const char *kLogRed = "\033[31m";
static constexpr const char *kLogYellow = "\033[33m";
static constexpr const char *kLogReset = "\033[0m";

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
        else if (lCommand == COMM_THERMOMETRY_ALARM)
        {
            if (dwBufLen < sizeof(NET_DVR_THERMOMETRY_ALARM))
            {
                RCLCPP_WARN(get_logger(), "[海康] 测温报警数据长度不足：实际=%u 期望至少=%zu", dwBufLen, sizeof(NET_DVR_THERMOMETRY_ALARM));
                return;
            }

            const auto *alarm = reinterpret_cast<const NET_DVR_THERMOMETRY_ALARM *>(pAlarmInfo);
            msg.temperature = alarm->fCurrTemperature;
            msg.raw_summary = std::string("测温") + alarm_level_name(alarm->byAlarmLevel);
            if (alarm->byPicTransType == 0)
            {
                assign_image(msg.image_data, alarm->pPicBuff, alarm->dwPicLen);
                assign_image(msg.thermal_image_data, alarm->pThermalPicBuff, alarm->dwThermalPicLen);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "[海康] 测温报警图片使用URL传输，当前节点不支持下载URL图片。");
            }
            msg.has_image = !msg.image_data.empty() || !msg.thermal_image_data.empty();
            RCLCPP_INFO(get_logger(), "%s[海康] 测温事件：级别=%s 当前温度=%.1f°C 预警规则=%.1f°C 报警规则=%.1f°C 类型=%s 规则=%s 可见光=%zu字节 红外=%zu字节%s",
                        alarm_level_color(alarm->byAlarmLevel), alarm_level_name(alarm->byAlarmLevel).c_str(), alarm->fCurrTemperature, alarm->fRuleTemperature, alarm->fAlarmRuleTemperature, thermometry_type_name(alarm->byAlarmType).c_str(),
                        thermometry_rule_name(alarm->byAlarmRule).c_str(), msg.image_data.size(), msg.thermal_image_data.size(), kLogReset);
            pub_->publish(msg);
        }
        else if (lCommand == COMM_THERMOMETRY_DIFF_ALARM)
        {
            if (dwBufLen < sizeof(NET_DVR_THERMOMETRY_DIFF_ALARM))
            {
                RCLCPP_WARN(get_logger(), "[海康] 温差报警数据长度不足：实际=%u 期望至少=%zu", dwBufLen, sizeof(NET_DVR_THERMOMETRY_DIFF_ALARM));
                return;
            }

            const auto *alarm = reinterpret_cast<const NET_DVR_THERMOMETRY_DIFF_ALARM *>(pAlarmInfo);
            msg.temperature = alarm->fCurTemperatureDiff;
            msg.raw_summary = std::string("温差") + alarm_level_name(alarm->byAlarmLevel);
            if (alarm->byPicTransType == 0)
            {
                assign_image(msg.image_data, alarm->pPicBuff, alarm->dwPicLen);
                assign_image(msg.thermal_image_data, alarm->pThermalPicBuff, alarm->dwThermalPicLen);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "[海康] 温差报警图片使用URL传输，当前节点不支持下载URL图片。");
            }
            msg.has_image = !msg.image_data.empty() || !msg.thermal_image_data.empty();
            RCLCPP_INFO(get_logger(), "%s[海康] 温差事件：级别=%s 当前温差=%.1f°C 规则温差=%.1f°C 类型=%s 规则=%s 可见光=%zu字节 红外=%zu字节%s",
                        alarm_level_color(alarm->byAlarmLevel), alarm_level_name(alarm->byAlarmLevel).c_str(),
                        alarm->fCurTemperatureDiff, alarm->fRuleTemperatureDiff, thermometry_type_name(alarm->byAlarmType).c_str(),
                        thermometry_rule_name(alarm->byAlarmRule).c_str(), msg.image_data.size(), msg.thermal_image_data.size(), kLogReset);
            pub_->publish(msg);
        }
        else if (lCommand == COMM_VCA_ALARM)
        {
            msg.has_image = false;
            msg.temperature = 0.0f;
            msg.raw_summary = "智能检测报警";
            pub_->publish(msg);
        }
    }

private:
    static void assign_image(std::vector<uint8_t> &output, const char *buffer, DWORD length)
    {
        if (buffer == nullptr || length == 0)
            return;
        const auto *begin = reinterpret_cast<const uint8_t *>(buffer);
        output.assign(begin, begin + length);
    }

    static std::string alarm_level_name(BYTE level)
    {
        if (level == 0)
            return "预警";
        if (level == 1)
            return "报警";
        return "未知级别" + std::to_string(level);
    }

    static const char *alarm_level_color(BYTE level)
    {
        return level == 1 ? kLogRed : kLogYellow;
    }

    static std::string thermometry_type_name(BYTE type)
    {
        switch (type)
        {
        case 0:
            return "最高温";
        case 1:
            return "最低温";
        case 2:
            return "平均温";
        case 3:
            return "温差";
        case 4:
            return "温度突升";
        case 5:
            return "温度突降";
        default:
            return "未知类型" + std::to_string(type);
        }
    }

    static std::string thermometry_rule_name(BYTE rule)
    {
        if (rule == 0)
            return "大于";
        if (rule == 1)
            return "小于";
        return "未知规则" + std::to_string(rule);
    }

    std::string masked_password() const
    {
        if (password_.empty())
            return "<空>";
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
            get_logger(), "[海康] 启动配置：地址=%s 端口=%d 用户=%s 密码=%s 登录模式=%d SDK日志=%s",
            device_ip_.c_str(), device_port_, username_.c_str(), masked_password().c_str(), login_mode_, sdk_log_enable_ ? "开启" : "关闭");
        RCLCPP_INFO(
            get_logger(), "[海康] SDK路径：库目录=%s 组件目录=%s ssl=%s crypto=%s",
            HIK_SDK_LIB_DIR, HIK_SDK_COM_DIR, HIK_SDK_SSL_PATH, HIK_SDK_CRYPTO_PATH);

        if (std::strlen(HIK_SDK_LIB_DIR) > 0)
        {
            NET_DVR_LOCAL_SDK_PATH sdk_path{};
            std::snprintf(sdk_path.sPath, sizeof(sdk_path.sPath), "%s", HIK_SDK_LIB_DIR);
            const bool ok = NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SDK_PATH, &sdk_path);
            RCLCPP_INFO(get_logger(), "[海康] 设置SDK库目录%s：%s", ok ? "成功" : "失败", sdk_path.sPath);
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
            RCLCPP_INFO(get_logger(), "[海康] 设置crypto库%s：%s", ok ? "成功" : "失败", crypto_path.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "[海康] SDK库目录下未找到可用的 libcrypto。");
        }

        const std::string ssl_path = first_existing({HIK_SDK_SSL_PATH, std::string(HIK_SDK_LIB_DIR) + "/libssl.so.3",
                                                     std::string(HIK_SDK_LIB_DIR) + "/libssl.so.1.1", std::string(HIK_SDK_LIB_DIR) + "/libssl.so"});
        if (!ssl_path.empty())
        {
            const bool ok = NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SSLEAY_PATH, const_cast<char *>(ssl_path.c_str()));
            RCLCPP_INFO(get_logger(), "[海康] 设置ssl库%s：%s", ok ? "成功" : "失败", ssl_path.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "[海康] SDK库目录下未找到可用的 libssl。");
        }
        if (sdk_log_enable_)
        {
            NET_DVR_SetLogToFile(3, const_cast<char *>(sdk_log_dir_.c_str()), FALSE);
        }

        if (!NET_DVR_Init())
        {
            RCLCPP_ERROR(get_logger(), "[海康] SDK初始化失败：错误码=%u", NET_DVR_GetLastError());
            rclcpp::shutdown();
            return;
        }

        NET_DVR_SetConnectTime(2000, 1);
        NET_DVR_SetReconnect(10000, true);

        NET_DVR_USER_LOGIN_INFO login_info{};
        NET_DVR_DEVICEINFO_V40 device_info{};
        std::snprintf(reinterpret_cast<char *>(login_info.sDeviceAddress), sizeof(login_info.sDeviceAddress), "%s", device_ip_.c_str());
        login_info.wPort = static_cast<WORD>(device_port_);
        std::snprintf(reinterpret_cast<char *>(login_info.sUserName), sizeof(login_info.sUserName), "%s", username_.c_str());
        std::snprintf(reinterpret_cast<char *>(login_info.sPassword), sizeof(login_info.sPassword), "%s", password_.c_str());
        login_info.byLoginMode = static_cast<BYTE>(login_mode_);
        login_info.byHttps = 0;

        user_id_ = NET_DVR_Login_V40(&login_info, &device_info);
        if (user_id_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "[海康] 登录设备失败：错误码=%u", NET_DVR_GetLastError());
            NET_DVR_Cleanup();
            rclcpp::shutdown();
            return;
        }

        if (!NET_DVR_SetDVRMessageCallBack_V30(MsgCallback, nullptr))
        {
            RCLCPP_ERROR(get_logger(), "[海康] 设置报警回调失败：错误码=%u", NET_DVR_GetLastError());
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
            RCLCPP_ERROR(get_logger(), "[海康] 布防失败：错误码=%u", NET_DVR_GetLastError());
            cleanup();
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "[海康] 报警订阅已启动。");
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
                                   { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "当前架构未启用海康SDK，请在支持的目标平台构建/运行以启用摄像机报警。"); });
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
