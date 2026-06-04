#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#ifdef USE_HIK_SDK
#include <cstring>

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
#endif

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
        save_pic_dir_ = declare_parameter<std::string>("save_pic_dir", "alarms");
        test_alarm_hold_seconds_ = declare_parameter<int>("test_alarm_hold_seconds", 5);

        const auto pkg_share = ament_index_cpp::get_package_share_directory("thermal_camera_monitor");
        save_pic_dir_ = resolve_save_dir(pkg_share, save_pic_dir_);
        std::filesystem::create_directories(save_pic_dir_);

        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/monitor/thermal_camera/status", 10);
        start_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/start", std::bind(&HikAlarmNode::on_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/stop", std::bind(&HikAlarmNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));
        test_alarm_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/test_alarm", std::bind(&HikAlarmNode::on_test_alarm, this, std::placeholders::_1, std::placeholders::_2));

        publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机未启动");
        RCLCPP_INFO(get_logger(), "热成像相机服务已就绪：start=/monitor/thermal_camera/start stop=/monitor/thermal_camera/stop status=/monitor/thermal_camera/status 地址=%s 图片目录=%s",
                    device_ip_.c_str(), save_pic_dir_.c_str());
    }

    ~HikAlarmNode() override
    {
        monitoring_active_ = false;
        if (worker_.joinable())
            worker_.join();
        cleanup();
    }

private:
    static constexpr const char *kLogRed = "\033[31m";
    static constexpr const char *kLogYellow = "\033[33m";
    static constexpr const char *kLogReset = "\033[0m";

    void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(thread_mutex_);
        if (monitoring_active_)
        {
            response->success = true;
            response->message = "热成像相机已在运行";
            return;
        }

        if (worker_.joinable())
            worker_.join();

        reset_latest_status();
        std::string start_message;
        if (!establish_session(start_message))
        {
            response->success = false;
            response->message = start_message;
            return;
        }

        monitoring_active_ = true;
        worker_ = std::thread(&HikAlarmNode::status_loop, this);
        response->success = true;
        response->message = "热成像相机已启动";
    }

    void on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(thread_mutex_);
            if (!monitoring_active_)
            {
                response->success = true;
                response->message = "热成像相机已停止";
                publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机已停止");
                return;
            }
            monitoring_active_ = false;
        }

        if (worker_.joinable())
            worker_.join();

        response->success = true;
        response->message = "热成像相机已停止";
        publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机已停止");
    }

    void on_test_alarm(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!monitoring_active_)
        {
            response->success = false;
            response->message = "热成像相机未启动，无法触发测试报警";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            latest_values_.clear();
            latest_artifacts_.clear();
            latest_alarm_active_ = true;
            latest_alarm_type_ = "thermal_camera_test_alarm";
            latest_alarm_detail_ = "热成像相机测试报警";
            latest_status_level_ = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            latest_values_.push_back(kv("test_alarm", "true"));
            latest_values_.push_back(kv("trigger_source", "service"));
            manual_test_alarm_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, test_alarm_hold_seconds_));
        }

        publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "热成像相机测试报警");
        response->success = true;
        response->message = "已触发热成像相机测试报警";
    }

    static diagnostic_msgs::msg::KeyValue kv(const std::string &key, const std::string &value)
    {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        return item;
    }

    static std::string number(float value)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << value;
        return oss.str();
    }

    static std::string resolve_save_dir(const std::string &pkg_share, const std::string &param_path)
    {
        std::filesystem::path p(param_path);
        if (p.is_absolute())
            return p.string();

        const std::filesystem::path share_path(pkg_share);
        std::filesystem::path workspace_root;
        for (auto it = share_path.begin(); it != share_path.end(); ++it)
        {
            if (*it == "install")
                break;
            workspace_root /= *it;
        }

        if (!workspace_root.empty() && std::filesystem::exists(workspace_root / "src" / "thermal_camera_monitor"))
            return (workspace_root / "src" / "thermal_camera_monitor" / p).string();

        return (share_path / p).string();
    }

    void reset_latest_status()
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        latest_values_.clear();
        latest_artifacts_.clear();
        latest_alarm_active_ = false;
        latest_alarm_type_.clear();
        latest_alarm_detail_.clear();
        latest_status_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
        manual_test_alarm_until_ = std::chrono::steady_clock::time_point{};
    }

    void refresh_manual_test_alarm_locked()
    {
        if (manual_test_alarm_until_ == std::chrono::steady_clock::time_point{})
            return;

        if (std::chrono::steady_clock::now() < manual_test_alarm_until_)
            return;

        if (latest_alarm_type_ == "thermal_camera_test_alarm")
        {
            latest_alarm_active_ = false;
            latest_alarm_type_.clear();
            latest_alarm_detail_.clear();
            latest_status_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
            latest_values_.clear();
            latest_artifacts_.clear();
        }
        manual_test_alarm_until_ = std::chrono::steady_clock::time_point{};
    }

    std::vector<diagnostic_msgs::msg::KeyValue> base_values_locked() const
    {
        std::vector<diagnostic_msgs::msg::KeyValue> values;
        values.push_back(kv("device_type", "thermal_camera"));
        values.push_back(kv("device_ip", device_ip_));
        values.push_back(kv("device_port", std::to_string(device_port_)));
        values.push_back(kv("login_mode", std::to_string(login_mode_)));
        values.push_back(kv("save_pic_dir", save_pic_dir_));
        values.push_back(kv("alarm_active", latest_alarm_active_ ? "true" : "false"));
        values.push_back(kv("alarm_type", latest_alarm_type_));
        values.push_back(kv("alarm_detail", latest_alarm_detail_));
        values.push_back(kv("artifact_count", std::to_string(latest_artifacts_.size())));
        for (size_t i = 0; i < latest_artifacts_.size(); ++i)
            values.push_back(kv("artifact_path_" + std::to_string(i), latest_artifacts_[i]));
        values.insert(values.end(), latest_values_.begin(), latest_values_.end());
        return values;
    }

    void publish_status(uint8_t level, const std::string &message)
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.level = level;
        status.name = "thermal_camera";
        status.message = message;
        status.hardware_id = device_ip_ + ":" + std::to_string(device_port_);

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            refresh_manual_test_alarm_locked();
            status.values = base_values_locked();
        }
        status_pub_->publish(status);
    }

    std::string save_image_from_buffer(const std::vector<uint8_t> &data, const std::string &prefix)
    {
        int offset = -1;
        for (size_t i = 0; i + 1 < data.size(); i++)
        {
            if (data[i] == 0xFF && data[i + 1] == 0xD8)
            {
                offset = static_cast<int>(i);
                break;
            }
        }
        if (offset < 0)
        {
            RCLCPP_WARN(get_logger(), "热成像报警数据中未找到 JPEG 起始标记，跳过保存图片。");
            return "";
        }

        const auto now_time = std::chrono::system_clock::now();
        const auto t = std::chrono::system_clock::to_time_t(now_time);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);

        std::ostringstream oss;
        oss << save_pic_dir_ << "/" << prefix << "_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".jpg";

        std::ofstream out(oss.str(), std::ios::binary);
        if (!out.is_open())
        {
            RCLCPP_WARN(get_logger(), "无法写入热成像报警图片：%s", oss.str().c_str());
            return "";
        }

        out.write(reinterpret_cast<const char *>(data.data() + offset), static_cast<std::streamsize>(data.size() - offset));
        out.close();
        return oss.str();
    }

#ifdef USE_HIK_SDK
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

    static bool is_rule_satisfied(float current_value, float threshold, BYTE rule)
    {
        if (rule == 0)
            return current_value >= threshold;
        if (rule == 1)
            return current_value <= threshold;
        return true;
    }

    static bool is_thermometry_alarm_active(float current_temperature, BYTE alarm_level, BYTE rule, float prewarning_threshold, float alarm_threshold)
    {
        const float threshold = alarm_level == 1 ? alarm_threshold : prewarning_threshold;
        return is_rule_satisfied(current_temperature, threshold, rule);
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

    static uint8_t diagnostic_level_from_alarm_level(BYTE alarm_level)
    {
        if (alarm_level == 0)
            return diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if (alarm_level == 1)
            return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    static std::string diagnostic_message_from_alarm_level(BYTE alarm_level)
    {
        if (alarm_level == 0)
            return "热成像预警";
        if (alarm_level == 1)
            return "热成像报警";
        return "热成像异常";
    }

    static std::string status_message_for_alarm(const std::string &alarm_type, uint8_t status_level)
    {
        const bool is_alarm = status_level == diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        const std::string suffix = is_alarm ? "报警" : "预警";

        if (alarm_type == "thermal_alarm")
            return "热成像测温" + suffix;
        if (alarm_type == "thermal_diff_alarm")
            return "热成像温差" + suffix;
        if (alarm_type == "camera_image_alarm")
            return "热成像智能检测" + suffix;
        if (alarm_type == "thermal_camera_test_alarm")
            return "热成像测试报警";
        return is_alarm ? "热成像报警" : "热成像预警";
    }

    std::string masked_password() const
    {
        if (password_.empty())
            return "<空>";
        if (password_.size() <= 2)
            return std::string(password_.size(), '*');
        return std::string(password_.size() - 2, '*') + password_.substr(password_.size() - 2);
    }

    void on_alarm(LONG lCommand, char *pAlarmInfo, DWORD dwBufLen)
    {
        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        std::string message = "收到热成像报警事件";

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            latest_values_.clear();
            latest_artifacts_.clear();
            latest_alarm_active_ = false;
            latest_alarm_type_.clear();
            latest_alarm_detail_.clear();
            latest_values_.push_back(kv("command", std::to_string(static_cast<int32_t>(lCommand))));

            if (lCommand == 0x4012)
            {
                const std::string raw(pAlarmInfo, pAlarmInfo + dwBufLen);
                latest_values_.push_back(kv("raw_summary", raw.substr(0, std::min<size_t>(raw.size(), 256))));
                latest_alarm_detail_ = raw.substr(0, std::min<size_t>(raw.size(), 256));
                message = "收到热成像原始事件";
            }
            else if (lCommand == COMM_THERMOMETRY_ALARM)
            {
                if (dwBufLen < sizeof(NET_DVR_THERMOMETRY_ALARM))
                {
                    latest_alarm_detail_ = "测温报警数据长度不足";
                    latest_values_.push_back(kv("buffer_length", std::to_string(dwBufLen)));
                    message = latest_alarm_detail_;
                    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                }
                else
                {
                    const auto *alarm = reinterpret_cast<const NET_DVR_THERMOMETRY_ALARM *>(pAlarmInfo);
                    std::vector<uint8_t> image_data;
                    std::vector<uint8_t> thermal_image_data;
                    if (alarm->byPicTransType == 0)
                    {
                        assign_image(image_data, alarm->pPicBuff, alarm->dwPicLen);
                        assign_image(thermal_image_data, alarm->pThermalPicBuff, alarm->dwThermalPicLen);
                    }
                    else
                    {
                        latest_values_.push_back(kv("pic_trans_type", std::to_string(alarm->byPicTransType)));
                    }

                    if (!image_data.empty())
                    {
                        const auto path = save_image_from_buffer(image_data, "visible");
                        if (!path.empty())
                            latest_artifacts_.push_back(path);
                    }
                    if (!thermal_image_data.empty())
                    {
                        const auto path = save_image_from_buffer(thermal_image_data, "thermal");
                        if (!path.empty())
                            latest_artifacts_.push_back(path);
                    }

                    latest_alarm_active_ = is_thermometry_alarm_active(alarm->fCurrTemperature, alarm->byAlarmLevel, alarm->byAlarmRule, alarm->fRuleTemperature, alarm->fAlarmRuleTemperature);
                    latest_alarm_type_ = "thermal_alarm";
                    latest_alarm_detail_ = std::string("测温") + alarm_level_name(alarm->byAlarmLevel);
                    latest_status_level_ = diagnostic_level_from_alarm_level(alarm->byAlarmLevel);
                    latest_values_.push_back(kv("temperature_c", number(alarm->fCurrTemperature)));
                    latest_values_.push_back(kv("alarm_level", std::to_string(alarm->byAlarmLevel)));
                    latest_values_.push_back(kv("alarm_type_code", std::to_string(alarm->byAlarmType)));
                    latest_values_.push_back(kv("alarm_type_name", thermometry_type_name(alarm->byAlarmType)));
                    latest_values_.push_back(kv("alarm_rule_code", std::to_string(alarm->byAlarmRule)));
                    latest_values_.push_back(kv("alarm_rule_name", thermometry_rule_name(alarm->byAlarmRule)));
                    latest_values_.push_back(kv("rule_temperature_c", number(alarm->fRuleTemperature)));
                    latest_values_.push_back(kv("alarm_rule_temperature_c", number(alarm->fAlarmRuleTemperature)));
                    message = status_message_for_alarm(latest_alarm_type_, latest_status_level_);
                    if (!latest_alarm_active_)
                        level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    else
                        level = latest_status_level_;

                    RCLCPP_INFO(get_logger(), "%s[海康] 测温事件：级别=%s 当前温度=%.1f°C 类型=%s 规则=%s 本地判定=%s 图片数=%zu%s",
                                alarm_level_color(alarm->byAlarmLevel), alarm_level_name(alarm->byAlarmLevel).c_str(), alarm->fCurrTemperature,
                                thermometry_type_name(alarm->byAlarmType).c_str(), thermometry_rule_name(alarm->byAlarmRule).c_str(),
                                latest_alarm_active_ ? "触发" : "未触发", latest_artifacts_.size(), kLogReset);
                }
            }
            else if (lCommand == COMM_THERMOMETRY_DIFF_ALARM)
            {
                if (dwBufLen < sizeof(NET_DVR_THERMOMETRY_DIFF_ALARM))
                {
                    latest_alarm_detail_ = "温差报警数据长度不足";
                    latest_values_.push_back(kv("buffer_length", std::to_string(dwBufLen)));
                    message = latest_alarm_detail_;
                    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                }
                else
                {
                    const auto *alarm = reinterpret_cast<const NET_DVR_THERMOMETRY_DIFF_ALARM *>(pAlarmInfo);
                    std::vector<uint8_t> image_data;
                    std::vector<uint8_t> thermal_image_data;
                    if (alarm->byPicTransType == 0)
                    {
                        assign_image(image_data, alarm->pPicBuff, alarm->dwPicLen);
                        assign_image(thermal_image_data, alarm->pThermalPicBuff, alarm->dwThermalPicLen);
                    }
                    else
                    {
                        latest_values_.push_back(kv("pic_trans_type", std::to_string(alarm->byPicTransType)));
                    }

                    if (!image_data.empty())
                    {
                        const auto path = save_image_from_buffer(image_data, "visible");
                        if (!path.empty())
                            latest_artifacts_.push_back(path);
                    }
                    if (!thermal_image_data.empty())
                    {
                        const auto path = save_image_from_buffer(thermal_image_data, "thermal");
                        if (!path.empty())
                            latest_artifacts_.push_back(path);
                    }

                    latest_alarm_active_ = is_rule_satisfied(alarm->fCurTemperatureDiff, alarm->fRuleTemperatureDiff, alarm->byAlarmRule);
                    latest_alarm_type_ = "thermal_diff_alarm";
                    latest_alarm_detail_ = std::string("温差") + alarm_level_name(alarm->byAlarmLevel);
                    latest_status_level_ = diagnostic_level_from_alarm_level(alarm->byAlarmLevel);
                    latest_values_.push_back(kv("temperature_diff_c", number(alarm->fCurTemperatureDiff)));
                    latest_values_.push_back(kv("alarm_level", std::to_string(alarm->byAlarmLevel)));
                    latest_values_.push_back(kv("alarm_type_code", std::to_string(alarm->byAlarmType)));
                    latest_values_.push_back(kv("alarm_type_name", thermometry_type_name(alarm->byAlarmType)));
                    latest_values_.push_back(kv("alarm_rule_code", std::to_string(alarm->byAlarmRule)));
                    latest_values_.push_back(kv("alarm_rule_name", thermometry_rule_name(alarm->byAlarmRule)));
                    latest_values_.push_back(kv("rule_temperature_diff_c", number(alarm->fRuleTemperatureDiff)));
                    message = status_message_for_alarm(latest_alarm_type_, latest_status_level_);
                    if (!latest_alarm_active_)
                        level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    else
                        level = latest_status_level_;

                    RCLCPP_INFO(get_logger(), "%s[海康] 温差事件：级别=%s 当前温差=%.1f°C 类型=%s 规则=%s 本地判定=%s 图片数=%zu%s",
                                alarm_level_color(alarm->byAlarmLevel), alarm_level_name(alarm->byAlarmLevel).c_str(), alarm->fCurTemperatureDiff,
                                thermometry_type_name(alarm->byAlarmType).c_str(), thermometry_rule_name(alarm->byAlarmRule).c_str(),
                                latest_alarm_active_ ? "触发" : "未触发", latest_artifacts_.size(), kLogReset);
                }
            }
            else if (lCommand == COMM_VCA_ALARM)
            {
                latest_alarm_active_ = true;
                latest_alarm_type_ = "camera_image_alarm";
                latest_alarm_detail_ = "智能检测报警";
                latest_status_level_ = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                message = status_message_for_alarm(latest_alarm_type_, latest_status_level_);
            }
        }

        publish_status(level, message);
    }

    static void CALLBACK MsgCallback(LONG lCommand, NET_DVR_ALARMER *pAlarmer, char *pAlarmInfo, DWORD dwBufLen, void *pUser)
    {
        (void)pAlarmer;
        if (pUser != nullptr && pAlarmInfo != nullptr && dwBufLen > 0)
            reinterpret_cast<HikAlarmNode *>(pUser)->on_alarm(lCommand, pAlarmInfo, dwBufLen);
    }

    bool establish_session(std::string &message)
    {
        RCLCPP_INFO(get_logger(), "[海康] 启动配置：地址=%s 端口=%d 用户=%s 密码=%s 登录模式=%d SDK日志=%s",
                    device_ip_.c_str(), device_port_, username_.c_str(), masked_password().c_str(), login_mode_, sdk_log_enable_ ? "开启" : "关闭");

        if (std::strlen(HIK_SDK_LIB_DIR) > 0)
        {
            NET_DVR_LOCAL_SDK_PATH sdk_path{};
            std::snprintf(sdk_path.sPath, sizeof(sdk_path.sPath), "%s", HIK_SDK_LIB_DIR);
            NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SDK_PATH, &sdk_path);
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

        const std::string crypto_path = first_existing({HIK_SDK_CRYPTO_PATH, std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so.3", std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so.1.1", std::string(HIK_SDK_LIB_DIR) + "/libcrypto.so"});
        if (!crypto_path.empty())
            NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_LIBEAY_PATH, const_cast<char *>(crypto_path.c_str()));

        const std::string ssl_path = first_existing({HIK_SDK_SSL_PATH, std::string(HIK_SDK_LIB_DIR) + "/libssl.so.3", std::string(HIK_SDK_LIB_DIR) + "/libssl.so.1.1", std::string(HIK_SDK_LIB_DIR) + "/libssl.so"});
        if (!ssl_path.empty())
            NET_DVR_SetSDKInitCfg(NET_SDK_INIT_CFG_SSLEAY_PATH, const_cast<char *>(ssl_path.c_str()));

        if (sdk_log_enable_)
            NET_DVR_SetLogToFile(3, const_cast<char *>(sdk_log_dir_.c_str()), FALSE);

        if (!NET_DVR_Init())
        {
            message = "SDK初始化失败，错误码=" + std::to_string(NET_DVR_GetLastError());
            publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
            return false;
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
            message = "登录设备失败，错误码=" + std::to_string(NET_DVR_GetLastError());
            publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
            cleanup();
            return false;
        }

        if (!NET_DVR_SetDVRMessageCallBack_V30(MsgCallback, this))
        {
            message = "设置报警回调失败，错误码=" + std::to_string(NET_DVR_GetLastError());
            publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
            cleanup();
            return false;
        }

        NET_DVR_SETUPALARM_PARAM setup_param{};
        setup_param.dwSize = sizeof(NET_DVR_SETUPALARM_PARAM);
        setup_param.byLevel = 0;
        setup_param.byAlarmInfoType = 1;

        alarm_handle_ = NET_DVR_SetupAlarmChan_V41(user_id_, &setup_param);
        if (alarm_handle_ < 0)
        {
            message = "布防失败，错误码=" + std::to_string(NET_DVR_GetLastError());
            publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
            cleanup();
            return false;
        }

        publish_status(diagnostic_msgs::msg::DiagnosticStatus::OK, "热成像相机报警订阅已启动");
        message = "热成像相机已启动";
        return true;
    }

    void status_loop()
    {
        while (rclcpp::ok() && monitoring_active_)
        {
            bool alarm_active = false;
            std::string alarm_type;
            uint8_t status_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                refresh_manual_test_alarm_locked();
                alarm_active = latest_alarm_active_;
                alarm_type = latest_alarm_type_;
                status_level = latest_status_level_;
            }
            publish_status(alarm_active ? status_level : diagnostic_msgs::msg::DiagnosticStatus::OK,
                           alarm_active ? status_message_for_alarm(alarm_type, status_level) : "热成像相机运行中");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        cleanup();
        publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机已停止");
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
#else
    bool establish_session(std::string &message)
    {
        message = "当前架构未启用海康SDK，请在支持的目标平台构建/运行。";
        publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
        return false;
    }

    void status_loop()
    {
        publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机已停止");
    }

    void cleanup() {}
#endif

    std::string device_ip_;
    int device_port_{};
    std::string username_;
    std::string password_;
    int login_mode_{};
    bool sdk_log_enable_{};
    std::string sdk_log_dir_;
    std::string save_pic_dir_;
    int test_alarm_hold_seconds_{};

    std::atomic<bool> monitoring_active_{false};
    std::thread worker_;
    std::mutex thread_mutex_;
    std::mutex status_mutex_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_alarm_srv_;

    std::vector<diagnostic_msgs::msg::KeyValue> latest_values_;
    std::vector<std::string> latest_artifacts_;
    bool latest_alarm_active_{false};
    std::string latest_alarm_type_;
    std::string latest_alarm_detail_;
    uint8_t latest_status_level_{diagnostic_msgs::msg::DiagnosticStatus::OK};
    std::chrono::steady_clock::time_point manual_test_alarm_until_{};

#ifdef USE_HIK_SDK
    LONG user_id_{-1};
    LONG alarm_handle_{-1};
#endif
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikAlarmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
