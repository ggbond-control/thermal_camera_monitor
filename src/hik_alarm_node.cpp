#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
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

namespace
{
    struct TemperatureGridStats
    {
        float min_temperature{0.0F};
        float max_temperature{0.0F};
        float avg_temperature{0.0F};
    };

    struct ParsedHeatmapFrame
    {
        int width{0};
        int height{0};
        bool freeze_data{false};
        std::vector<float> temperatures;
        std::string source_format;
    };

#pragma pack(push, 1)
    struct HikDateTime
    {
        uint16_t year;
        uint16_t month;
        uint16_t day_of_week;
        uint16_t day;
        uint16_t hour;
        uint16_t minute;
        uint16_t second;
        uint16_t millisecond;
    };

    struct StreamRtDataInfoTemp
    {
        uint32_t rt_data_type;
        uint32_t frame_num;
        uint32_t std_stamp;
        HikDateTime time;
        uint32_t width;
        uint32_t height;
        uint32_t length;
        uint32_t fps;
        uint32_t channel;
    };

    struct StreamFsSuppleInfoTemp
    {
        uint32_t tm_data_mode;
        uint32_t tm_scale;
        uint32_t tm_offset;
        uint32_t freeze_data;
    };

    struct StreamFrameInfoTemp
    {
        uint32_t magic_no;
        uint32_t header_size;
        uint32_t stream_type;
        uint32_t stream_len;
        StreamRtDataInfoTemp rt_info;
        StreamFsSuppleInfoTemp supplementary;
        uint32_t reserved[12];
        uint32_t crc;
    };
#pragma pack(pop)

    constexpr uint32_t kFullScreenThermometryResultType = 2;
    constexpr size_t kHeatmapJpegBufferSize = 8U * 1024U * 1024U;
    constexpr size_t kHeatmapP2pBufferSize = 8U * 1024U * 1024U;
    constexpr size_t kHeatmapHeaderSearchBytes = 512U;
} // namespace

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
        test_alarm_hold_seconds_ = declare_parameter<int>("test_alarm_hold_seconds", 1);
        thermometry_channel_ = declare_parameter<int>("thermometry_channel", 1);
        realtime_poll_interval_ms_ = declare_parameter<int>("realtime_poll_interval_ms", 1000);
        heatmap_enable_ = declare_parameter<bool>("heatmap_enable", true);
        heatmap_channel_ = declare_parameter<int>("heatmap_channel", 2);
        heatmap_grid_rows_ = declare_parameter<int>("heatmap_grid_rows", 2);
        heatmap_grid_cols_ = declare_parameter<int>("heatmap_grid_cols", 3);
        active_thermometry_channel_ = thermometry_channel_;
        heatmap_grid_rows_ = std::max(1, heatmap_grid_rows_);
        heatmap_grid_cols_ = std::max(1, heatmap_grid_cols_);

        const auto pkg_share = ament_index_cpp::get_package_share_directory("thermal_camera_monitor");
        save_pic_dir_ = resolve_save_dir(pkg_share, save_pic_dir_);
        std::filesystem::create_directories(save_pic_dir_);

        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/monitor/thermal_camera/status", 10);
        heatmap_pub_ = create_publisher<sensor_msgs::msg::Image>("/monitor/thermal_camera/heatmap", 10);
        start_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/start", std::bind(&HikAlarmNode::on_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/stop", std::bind(&HikAlarmNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));
        test_alarm_srv_ = create_service<std_srvs::srv::Trigger>("/monitor/thermal_camera/test_alarm", std::bind(&HikAlarmNode::on_test_alarm, this, std::placeholders::_1, std::placeholders::_2));

        publish_status(diagnostic_msgs::msg::DiagnosticStatus::STALE, "热成像相机未启动");
        RCLCPP_INFO(get_logger(), "热成像相机服务已就绪：start=/monitor/thermal_camera/start stop=/monitor/thermal_camera/stop status=/monitor/thermal_camera/status 地址=%s 图片目录=%s 实时测温通道=%d 热力图通道=%d",
                    device_ip_.c_str(), save_pic_dir_.c_str(), thermometry_channel_, heatmap_channel_);
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
        RCLCPP_ERROR(get_logger(), "[海康] 已触发测试报警：level=ERROR(2) 持续=%d秒", std::max(1, test_alarm_hold_seconds_));
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

    static std::string number_array_json(const std::vector<float> &values)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i)
                oss << ",";
            oss << std::fixed << std::setprecision(2) << values[i];
        }
        return oss.str();
    }

    static std::string preview_hex(const char *data, size_t length, size_t max_bytes = 16)
    {
        if (data == nullptr || length == 0)
            return "";
        std::ostringstream oss;
        const size_t count = std::min(length, max_bytes);
        for (size_t i = 0; i < count; ++i)
        {
            if (i)
                oss << ' ';
            oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(static_cast<unsigned char>(data[i]));
        }
        return oss.str();
    }

    static std::string trim_for_log(const std::string &text, size_t max_chars = 240)
    {
        if (text.size() <= max_chars)
            return text;
        return text.substr(0, max_chars) + "...";
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
        realtime_thermometry_valid_ = false;
        realtime_temperature_c_ = 0.0F;
        realtime_max_temperature_c_ = 0.0F;
        realtime_min_temperature_c_ = 0.0F;
        realtime_avg_temperature_c_ = 0.0F;
        realtime_temperature_diff_c_ = 0.0F;
        realtime_rule_name_ = "realtime_thermometry";
        realtime_rule_id_ = 0;
        realtime_calib_type_ = 2;
        realtime_error_.clear();
        region_avg_temperatures_c_.clear();
        region_max_temperatures_c_.clear();
        region_min_temperatures_c_.clear();
        active_thermometry_channel_ = thermometry_channel_;
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
        values.push_back(kv("thermometry_channel", std::to_string(thermometry_channel_)));
        values.push_back(kv("alarm_active", latest_alarm_active_ ? "true" : "false"));
        values.push_back(kv("alarm_type", latest_alarm_type_));
        values.push_back(kv("alarm_detail", latest_alarm_detail_));
        values.push_back(kv("realtime_valid", realtime_thermometry_valid_ ? "true" : "false"));
        values.push_back(kv("realtime_error", realtime_error_));
        values.push_back(kv("temperature_c", realtime_thermometry_valid_ ? number(realtime_temperature_c_) : ""));
        values.push_back(kv("max_temperature_c", realtime_thermometry_valid_ ? number(realtime_max_temperature_c_) : ""));
        values.push_back(kv("min_temperature_c", realtime_thermometry_valid_ ? number(realtime_min_temperature_c_) : ""));
        values.push_back(kv("avg_temperature_c", realtime_thermometry_valid_ ? number(realtime_avg_temperature_c_) : ""));
        values.push_back(kv("temperature_diff_c", realtime_thermometry_valid_ ? number(realtime_temperature_diff_c_) : ""));
        values.push_back(kv("realtime_rule_name", realtime_rule_name_));
        values.push_back(kv("realtime_rule_id", std::to_string(realtime_rule_id_)));
        values.push_back(kv("realtime_calib_type", std::to_string(realtime_calib_type_)));
        values.push_back(kv("region_temperature_rows", std::to_string(heatmap_grid_rows_)));
        values.push_back(kv("region_temperature_cols", std::to_string(heatmap_grid_cols_)));
        values.push_back(kv("region_temperature_count", std::to_string(heatmap_grid_rows_ * heatmap_grid_cols_)));
        values.push_back(kv("region_avg_temperature_c", number_array_json(region_avg_temperatures_c_)));
        values.push_back(kv("region_max_temperature_c", number_array_json(region_max_temperatures_c_)));
        values.push_back(kv("region_min_temperature_c", number_array_json(region_min_temperatures_c_)));
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

    void publish_heatmap(const cv::Mat &image)
    {
        if (image.empty() || image.type() != CV_8UC3)
            return;

        cv::Mat contiguous = image;
        if (!contiguous.isContinuous())
            contiguous = image.clone();

        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.height = static_cast<uint32_t>(contiguous.rows);
        msg.width = static_cast<uint32_t>(contiguous.cols);
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
        msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.step);
        const size_t data_size = contiguous.total() * contiguous.elemSize();
        msg.data.assign(contiguous.datastart, contiguous.datastart + data_size);
        heatmap_pub_->publish(msg);
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
                latest_values_.push_back(kv("raw_length", std::to_string(dwBufLen)));
                latest_values_.push_back(kv("raw_preview_hex", preview_hex(pAlarmInfo, dwBufLen)));
                latest_alarm_detail_ = "热成像原始事件";
                message = "热成像原始事件";
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

    void update_realtime_thermometry_locked(float max_temperature,
                                            float min_temperature,
                                            float average_temperature,
                                            float temperature_diff)
    {
        realtime_thermometry_valid_ = true;
        realtime_rule_id_ = 0;
        realtime_rule_name_ = "realtime_thermometry";
        realtime_calib_type_ = 2;
        realtime_temperature_c_ = average_temperature;
        realtime_max_temperature_c_ = max_temperature;
        realtime_min_temperature_c_ = min_temperature;
        realtime_avg_temperature_c_ = average_temperature;
        realtime_temperature_diff_c_ = temperature_diff;
        realtime_error_.clear();
    }

    void invalidate_realtime_thermometry_locked()
    {
        realtime_thermometry_valid_ = false;
        realtime_temperature_c_ = 0.0F;
        realtime_max_temperature_c_ = 0.0F;
        realtime_min_temperature_c_ = 0.0F;
        realtime_avg_temperature_c_ = 0.0F;
        realtime_temperature_diff_c_ = 0.0F;
        realtime_rule_name_ = "realtime_thermometry";
        realtime_rule_id_ = 0;
        realtime_calib_type_ = 2;
    }

    static bool is_plausible_heatmap_header(const StreamFrameInfoTemp &frame_info,
                                            size_t available_bytes,
                                            uint64_t &sample_count,
                                            uint32_t &sample_bytes,
                                            std::string &reject_reason)
    {
        if (frame_info.rt_info.rt_data_type != 1 && frame_info.rt_info.rt_data_type != 2 && frame_info.rt_info.rt_data_type != 3)
        {
            reject_reason = "unsupported rt data type=" + std::to_string(frame_info.rt_info.rt_data_type);
            return false;
        }
        if (frame_info.header_size < sizeof(StreamFrameInfoTemp) || frame_info.header_size > available_bytes)
        {
            reject_reason = "invalid header size=" + std::to_string(frame_info.header_size);
            return false;
        }
        if (frame_info.rt_info.width == 0 || frame_info.rt_info.height == 0)
        {
            reject_reason = "invalid heatmap size";
            return false;
        }
        if (frame_info.rt_info.width > 10000 || frame_info.rt_info.height > 10000)
        {
            reject_reason = "heatmap size too large";
            return false;
        }

        sample_count = static_cast<uint64_t>(frame_info.rt_info.width) * static_cast<uint64_t>(frame_info.rt_info.height);
        if (sample_count == 0 || sample_count > (64ULL * 1024ULL * 1024ULL))
        {
            reject_reason = "invalid sample count";
            return false;
        }

        if (frame_info.supplementary.tm_data_mode == 0)
            sample_bytes = 4U;
        else if (frame_info.supplementary.tm_data_mode == 1)
            sample_bytes = 2U;
        else
        {
            reject_reason = "unsupported tm data mode=" + std::to_string(frame_info.supplementary.tm_data_mode);
            return false;
        }

        if (frame_info.supplementary.tm_scale == 0)
        {
            reject_reason = "invalid tm scale=0";
            return false;
        }

        const uint64_t required_size = static_cast<uint64_t>(frame_info.header_size) + sample_count * sample_bytes;
        if (required_size > available_bytes)
        {
            reject_reason = "payload too short";
            return false;
        }

        return true;
    }

    static bool infer_raw_float_heatmap_shape(size_t sample_count,
                                              int jpeg_width,
                                              int jpeg_height,
                                              int &inferred_width,
                                              int &inferred_height)
    {
        if (sample_count == 0)
            return false;

        const double target_aspect = (jpeg_width > 0 && jpeg_height > 0)
                                         ? static_cast<double>(jpeg_width) / static_cast<double>(jpeg_height)
                                         : 4.0 / 3.0;

        double best_score = std::numeric_limits<double>::max();
        int best_width = 0;
        int best_height = 0;

        for (size_t h = 1; h * h <= sample_count; ++h)
        {
            if (sample_count % h != 0)
                continue;

            const size_t w = sample_count / h;
            const std::array<std::pair<int, int>, 2> candidates = {
                std::make_pair(static_cast<int>(w), static_cast<int>(h)),
                std::make_pair(static_cast<int>(h), static_cast<int>(w))};

            for (const auto &candidate : candidates)
            {
                const int cw = candidate.first;
                const int ch = candidate.second;
                if (cw <= 0 || ch <= 0 || cw > 4096 || ch > 4096)
                    continue;
                if (cw < 16 || ch < 16)
                    continue;

                const double aspect = static_cast<double>(cw) / static_cast<double>(ch);
                const double aspect_error = std::abs(aspect - target_aspect);
                const double squareness_penalty = std::abs(std::log(aspect));
                const double score = aspect_error + squareness_penalty * 0.05;
                if (score < best_score)
                {
                    best_score = score;
                    best_width = cw;
                    best_height = ch;
                }
            }
        }

        if (best_width == 0 || best_height == 0)
            return false;

        inferred_width = best_width;
        inferred_height = best_height;
        return true;
    }

    bool parse_heatmap_frame(const std::vector<uint8_t> &raw_buffer,
                             int jpeg_width,
                             int jpeg_height,
                             bool freeze_data,
                             ParsedHeatmapFrame &frame,
                             std::string &error_message)
    {
        frame = ParsedHeatmapFrame{};
        if (raw_buffer.size() < sizeof(StreamFrameInfoTemp))
            error_message = "p2p buffer too short for framed format";

        const size_t search_limit = raw_buffer.size() >= sizeof(StreamFrameInfoTemp)
                                        ? std::min(raw_buffer.size() - sizeof(StreamFrameInfoTemp), kHeatmapHeaderSearchBytes)
                                        : 0;
        const StreamFrameInfoTemp *frame_info = nullptr;
        size_t header_offset = 0;
        std::string last_reject_reason = "no candidate header";
        uint64_t sample_count = 0;
        uint32_t sample_bytes = 0;

        for (size_t offset = 0; offset <= search_limit; ++offset)
        {
            const auto *candidate = reinterpret_cast<const StreamFrameInfoTemp *>(raw_buffer.data() + offset);
            uint64_t candidate_sample_count = 0;
            uint32_t candidate_sample_bytes = 0;
            std::string reject_reason;
            if (!is_plausible_heatmap_header(*candidate,
                                             raw_buffer.size() - offset,
                                             candidate_sample_count,
                                             candidate_sample_bytes,
                                             reject_reason))
            {
                last_reject_reason = reject_reason;
                continue;
            }

            if (candidate->rt_info.rt_data_type != kFullScreenThermometryResultType)
            {
                last_reject_reason = "rt data type is not full-screen result, value=" + std::to_string(candidate->rt_info.rt_data_type);
                continue;
            }

            frame_info = candidate;
            header_offset = offset;
            sample_count = candidate_sample_count;
            sample_bytes = candidate_sample_bytes;
            break;
        }

        if (frame_info == nullptr)
        {
            if (raw_buffer.size() % sizeof(float) != 0)
            {
                error_message = "unable to locate valid p2p frame header, preview=" +
                                preview_hex(reinterpret_cast<const char *>(raw_buffer.data()), raw_buffer.size(), 32) +
                                " reason=" + last_reject_reason;
                return false;
            }

            const size_t sample_count_float = raw_buffer.size() / sizeof(float);
            int inferred_width = 0;
            int inferred_height = 0;
            if (!infer_raw_float_heatmap_shape(sample_count_float, jpeg_width, jpeg_height, inferred_width, inferred_height))
            {
                error_message = "unable to infer raw float heatmap shape, samples=" + std::to_string(sample_count_float) +
                                " preview=" + preview_hex(reinterpret_cast<const char *>(raw_buffer.data()), raw_buffer.size(), 32) +
                                " reason=" + last_reject_reason;
                return false;
            }

            frame.width = inferred_width;
            frame.height = inferred_height;
            frame.freeze_data = freeze_data;
            frame.source_format = "raw_float";
            frame.temperatures.resize(sample_count_float);

            size_t plausible_count = 0;
            for (size_t i = 0; i < sample_count_float; ++i)
            {
                float value = 0.0F;
                std::memcpy(&value, raw_buffer.data() + i * sizeof(float), sizeof(float));
                frame.temperatures[i] = value;
                if (std::isfinite(value) && value > -100.0F && value < 1000.0F)
                    ++plausible_count;
            }

            if (plausible_count < sample_count_float * 9 / 10)
            {
                error_message = "raw float heatmap values not plausible enough, samples=" + std::to_string(sample_count_float) +
                                " preview=" + preview_hex(reinterpret_cast<const char *>(raw_buffer.data()), raw_buffer.size(), 32) +
                                " reason=" + last_reject_reason;
                return false;
            }

            return true;
        }

        const uint32_t header_size = frame_info->header_size;
        frame.width = static_cast<int>(frame_info->rt_info.width);
        frame.height = static_cast<int>(frame_info->rt_info.height);
        frame.freeze_data = frame_info->supplementary.freeze_data != 0;
        frame.source_format = "framed";
        frame.temperatures.resize(static_cast<size_t>(sample_count));

        const uint8_t *payload = raw_buffer.data() + header_offset + header_size;
        const float offset = static_cast<float>(static_cast<int32_t>(frame_info->supplementary.tm_offset));
        const uint32_t scale = frame_info->supplementary.tm_scale;
        if (sample_bytes == 4U)
        {
            for (size_t i = 0; i < frame.temperatures.size(); ++i)
            {
                int32_t raw_value = 0;
                std::memcpy(&raw_value, payload + i * sample_bytes, sizeof(raw_value));
                frame.temperatures[i] = static_cast<float>(raw_value) / static_cast<float>(scale) + offset;
            }
        }
        else
        {
            for (size_t i = 0; i < frame.temperatures.size(); ++i)
            {
                uint16_t raw_value = 0;
                std::memcpy(&raw_value, payload + i * sample_bytes, sizeof(raw_value));
                frame.temperatures[i] = static_cast<float>(raw_value) / static_cast<float>(scale) + offset;
            }
        }

        return true;
    }

    bool capture_heatmap_frame(std::vector<uint8_t> &jpeg_buffer, ParsedHeatmapFrame &frame, std::string &error_message)
    {
        jpeg_buffer.clear();
        frame = ParsedHeatmapFrame{};

        std::vector<uint8_t> jpeg_storage(kHeatmapJpegBufferSize, 0);
        std::vector<uint8_t> p2p_storage(kHeatmapP2pBufferSize, 0);

        NET_DVR_JPEGPICTURE_WITH_APPENDDATA capture{};
        capture.dwSize = sizeof(capture);
        capture.dwChannel = static_cast<DWORD>(heatmap_channel_);
        capture.pJpegPicBuff = reinterpret_cast<char *>(jpeg_storage.data());
        capture.dwJpegPicLen = static_cast<DWORD>(jpeg_storage.size());
        capture.pP2PDataBuff = reinterpret_cast<char *>(p2p_storage.data());
        capture.dwP2PDataLen = static_cast<DWORD>(p2p_storage.size());

        if (!NET_DVR_CaptureJPEGPicture_WithAppendData(user_id_, heatmap_channel_, &capture))
        {
            error_message = "capture jpeg with append data failed, error=" + std::to_string(NET_DVR_GetLastError());
            return false;
        }
        if (capture.dwJpegPicLen == 0)
        {
            error_message = "jpeg buffer empty";
            return false;
        }
        if (capture.dwP2PDataLen == 0)
        {
            error_message = "p2p temperature buffer empty";
            return false;
        }
        if (capture.dwJpegPicLen > jpeg_storage.size() || capture.dwP2PDataLen > p2p_storage.size())
        {
            error_message = "captured buffer exceeds local storage";
            return false;
        }

        jpeg_buffer.assign(jpeg_storage.begin(), jpeg_storage.begin() + capture.dwJpegPicLen);
        std::vector<uint8_t> p2p_buffer(p2p_storage.begin(), p2p_storage.begin() + capture.dwP2PDataLen);
        if (!parse_heatmap_frame(p2p_buffer,
                                 static_cast<int>(capture.dwJpegPicWidth),
                                 static_cast<int>(capture.dwJpegPicHeight),
                                 capture.byIsFreezedata != 0,
                                 frame,
                                 error_message))
            return false;

        const bool should_log_dimensions = !heatmap_dimensions_logged_ ||
                                           last_heatmap_jpeg_width_ != static_cast<int>(capture.dwJpegPicWidth) ||
                                           last_heatmap_jpeg_height_ != static_cast<int>(capture.dwJpegPicHeight) ||
                                           last_heatmap_frame_width_ != frame.width ||
                                           last_heatmap_frame_height_ != frame.height ||
                                           last_heatmap_source_format_ != frame.source_format;
        if (should_log_dimensions)
        {
            heatmap_dimensions_logged_ = true;
            last_heatmap_jpeg_width_ = static_cast<int>(capture.dwJpegPicWidth);
            last_heatmap_jpeg_height_ = static_cast<int>(capture.dwJpegPicHeight);
            last_heatmap_frame_width_ = frame.width;
            last_heatmap_frame_height_ = frame.height;
            last_heatmap_source_format_ = frame.source_format;
            RCLCPP_INFO(get_logger(),
                        "[海康] 热力图抓图成功：channel=%d sdk_jpeg=%ux%u p2p_len=%u frame=%dx%d format=%s freeze=%s",
                        heatmap_channel_,
                        capture.dwJpegPicWidth,
                        capture.dwJpegPicHeight,
                        capture.dwP2PDataLen,
                        frame.width,
                        frame.height,
                        frame.source_format.c_str(),
                        frame.freeze_data ? "true" : "false");
        }

        return true;
    }

    std::vector<TemperatureGridStats> compute_heatmap_grid(const ParsedHeatmapFrame &frame) const
    {
        std::vector<TemperatureGridStats> grid;
        grid.reserve(static_cast<size_t>(heatmap_grid_rows_ * heatmap_grid_cols_));

        for (int row = 0; row < heatmap_grid_rows_; ++row)
        {
            const int y0 = row * frame.height / heatmap_grid_rows_;
            const int y1 = (row + 1) * frame.height / heatmap_grid_rows_;
            for (int col = 0; col < heatmap_grid_cols_; ++col)
            {
                const int x0 = col * frame.width / heatmap_grid_cols_;
                const int x1 = (col + 1) * frame.width / heatmap_grid_cols_;

                float min_temperature = std::numeric_limits<float>::max();
                float max_temperature = std::numeric_limits<float>::lowest();
                float sum_temperature = 0.0F;
                size_t count = 0;

                for (int y = y0; y < y1; ++y)
                {
                    for (int x = x0; x < x1; ++x)
                    {
                        const float value = frame.temperatures[static_cast<size_t>(y * frame.width + x)];
                        min_temperature = std::min(min_temperature, value);
                        max_temperature = std::max(max_temperature, value);
                        sum_temperature += value;
                        ++count;
                    }
                }

                TemperatureGridStats stats;
                if (count > 0)
                {
                    stats.min_temperature = min_temperature;
                    stats.max_temperature = max_temperature;
                    stats.avg_temperature = sum_temperature / static_cast<float>(count);
                }
                grid.push_back(stats);
            }
        }

        return grid;
    }

    void update_region_thermometry_status(const std::vector<TemperatureGridStats> &grid)
    {
        std::vector<float> avg_values;
        std::vector<float> max_values;
        std::vector<float> min_values;
        avg_values.reserve(grid.size());
        max_values.reserve(grid.size());
        min_values.reserve(grid.size());

        for (const auto &cell : grid)
        {
            avg_values.push_back(cell.avg_temperature);
            max_values.push_back(cell.max_temperature);
            min_values.push_back(cell.min_temperature);
        }

        std::lock_guard<std::mutex> lock(status_mutex_);
        region_avg_temperatures_c_ = std::move(avg_values);
        region_max_temperatures_c_ = std::move(max_values);
        region_min_temperatures_c_ = std::move(min_values);
    }

    void clear_region_thermometry_status()
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        region_avg_temperatures_c_.clear();
        region_max_temperatures_c_.clear();
        region_min_temperatures_c_.clear();
    }

    cv::Mat render_heatmap_overlay(const std::vector<uint8_t> &jpeg_buffer,
                                   const ParsedHeatmapFrame &frame,
                                   const std::vector<TemperatureGridStats> &grid,
                                   std::string &error_message) const
    {
        cv::Mat decoded = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
        if (decoded.empty())
        {
            error_message = "failed to decode thermal jpeg";
            return {};
        }

        const double scale_x = static_cast<double>(decoded.cols) / static_cast<double>(frame.width);
        const double scale_y = static_cast<double>(decoded.rows) / static_cast<double>(frame.height);
        const int cell_count = heatmap_grid_rows_ * heatmap_grid_cols_;
        if (static_cast<int>(grid.size()) != cell_count)
        {
            error_message = "heatmap grid size mismatch";
            return {};
        }

        const double cell_width = static_cast<double>(decoded.cols) / static_cast<double>(heatmap_grid_cols_);
        const double cell_height = static_cast<double>(decoded.rows) / static_cast<double>(heatmap_grid_rows_);
        const double font_scale_from_cell = std::min(cell_width / 140.0, cell_height / 85.0);
        const double font_scale = std::clamp(font_scale_from_cell, 0.18, 0.32);
        const int thickness = 1;
        const cv::Scalar line_color(0, 255, 255);
        const cv::Scalar text_color(255, 255, 255);
        const cv::Scalar shadow_color(0, 0, 0);

        for (int row = 0; row < heatmap_grid_rows_; ++row)
        {
            const int src_y0 = row * frame.height / heatmap_grid_rows_;
            const int src_y1 = (row + 1) * frame.height / heatmap_grid_rows_;
            const int dst_y0 = std::clamp(static_cast<int>(std::round(src_y0 * scale_y)), 0, decoded.rows - 1);
            const int dst_y1 = std::clamp(static_cast<int>(std::round(src_y1 * scale_y)), dst_y0 + 1, decoded.rows);

            for (int col = 0; col < heatmap_grid_cols_; ++col)
            {
                const int index = row * heatmap_grid_cols_ + col;
                const int src_x0 = col * frame.width / heatmap_grid_cols_;
                const int src_x1 = (col + 1) * frame.width / heatmap_grid_cols_;
                const int dst_x0 = std::clamp(static_cast<int>(std::round(src_x0 * scale_x)), 0, decoded.cols - 1);
                const int dst_x1 = std::clamp(static_cast<int>(std::round(src_x1 * scale_x)), dst_x0 + 1, decoded.cols);

                cv::rectangle(decoded, cv::Point(dst_x0, dst_y0), cv::Point(dst_x1 - 1, dst_y1 - 1), line_color, 1);

                const std::array<std::string, 3> labels = {
                    "AVG " + number(grid[static_cast<size_t>(index)].avg_temperature),
                    "MAX " + number(grid[static_cast<size_t>(index)].max_temperature),
                    "MIN " + number(grid[static_cast<size_t>(index)].min_temperature)};

                const int top_padding = 3;
                const int left_padding = 2;
                const int baseline_step = std::max(8, static_cast<int>(std::round(cell_height / 5.0)));
                int text_y = std::min(dst_y0 + top_padding + baseline_step, decoded.rows - 4);
                const int text_x = std::min(dst_x0 + left_padding, std::max(0, decoded.cols - 1));
                for (const auto &label : labels)
                {
                    cv::putText(decoded, label, cv::Point(text_x + 1, text_y + 1), cv::FONT_HERSHEY_SIMPLEX, font_scale, shadow_color, thickness + 1, cv::LINE_AA);
                    cv::putText(decoded, label, cv::Point(text_x, text_y), cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness, cv::LINE_AA);
                    text_y = std::min(text_y + baseline_step, decoded.rows - 6);
                }
            }
        }

        return decoded;
    }

    void update_region_thermometry_and_optional_heatmap()
    {
        std::vector<uint8_t> jpeg_buffer;
        ParsedHeatmapFrame frame;
        std::string error_message;
        if (!capture_heatmap_frame(jpeg_buffer, frame, error_message))
        {
            clear_region_thermometry_status();
            RCLCPP_WARN(get_logger(), "[海康] 热力分布图抓取失败：channel=%d error=%s", heatmap_channel_, error_message.c_str());
            return;
        }

        const auto grid = compute_heatmap_grid(frame);
        update_region_thermometry_status(grid);
        if (!heatmap_enable_)
            return;

        cv::Mat rendered = render_heatmap_overlay(jpeg_buffer, frame, grid, error_message);
        if (rendered.empty())
        {
            RCLCPP_WARN(get_logger(), "[海康] 热力分布图渲染失败：channel=%d error=%s", heatmap_channel_, error_message.c_str());
            return;
        }

        publish_heatmap(rendered);
    }

    bool fetch_rule_temperature_info(int channel, NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO &temperature_info, std::string &error_message)
    {
        DWORD bytes_returned = 0;
        std::memset(&temperature_info, 0, sizeof(temperature_info));
        if (NET_DVR_GetDVRConfig(user_id_, NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO, channel, &temperature_info, sizeof(temperature_info), &bytes_returned))
        {
            if (bytes_returned >= sizeof(temperature_info))
                return true;
            error_message = "get rule temperature returned too few bytes";
            return false;
        }

        error_message = "get rule temperature failed, error=" + std::to_string(NET_DVR_GetLastError());
        return false;
    }

    enum class RealtimeQueryResult
    {
        kUpdated,
        kNoData,
        kFailed,
    };

    RealtimeQueryResult query_sdk_realtime_thermometry()
    {
        NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO temperature_info{};
        std::string query_error;
        if (fetch_rule_temperature_info(thermometry_channel_, temperature_info, query_error))
        {
            const float temperature_diff = temperature_info.fMaxTemperature - temperature_info.fMinTemperature;
            std::lock_guard<std::mutex> lock(status_mutex_);
            active_thermometry_channel_ = thermometry_channel_;
            update_realtime_thermometry_locked(temperature_info.fMaxTemperature,
                                               temperature_info.fMinTemperature,
                                               temperature_info.fAverageTemperature,
                                               temperature_diff);
            return RealtimeQueryResult::kUpdated;
        }

        RCLCPP_WARN(get_logger(), "[海康] SDK实时测温查询失败：channel=%d error=%s", thermometry_channel_, query_error.c_str());
        std::lock_guard<std::mutex> lock(status_mutex_);
        active_thermometry_channel_ = thermometry_channel_;
        realtime_error_ = query_error;
        return RealtimeQueryResult::kFailed;
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
            const auto realtime_result = query_sdk_realtime_thermometry();
            if (realtime_result == RealtimeQueryResult::kFailed)
            {
                std::string realtime_error;
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    invalidate_realtime_thermometry_locked();
                    realtime_error = realtime_error_;
                }
                RCLCPP_WARN(get_logger(), "SDK实时测温未获取到有效数据：%s", realtime_error.c_str());
            }
            else if (realtime_result == RealtimeQueryResult::kUpdated)
            {
                float avg_temperature = 0.0F;
                float max_temperature = 0.0F;
                float min_temperature = 0.0F;
                float temperature_diff = 0.0F;
                int active_channel = -1;
                std::string rule_name;
                uint8_t rule_id = 0;
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    avg_temperature = realtime_avg_temperature_c_;
                    max_temperature = realtime_max_temperature_c_;
                    min_temperature = realtime_min_temperature_c_;
                    temperature_diff = realtime_temperature_diff_c_;
                    active_channel = active_thermometry_channel_;
                    rule_name = realtime_rule_name_;
                    rule_id = realtime_rule_id_;
                }
                RCLCPP_INFO(
                    get_logger(),
                    "[海康] 实时测温：channel=%d rule_id=%u rule_name=%s 平均=%.1f°C 最高=%.1f°C 最低=%.1f°C 温差=%.1f°C",
                    active_channel, rule_id, rule_name.c_str(), avg_temperature, max_temperature, min_temperature, temperature_diff);
                update_region_thermometry_and_optional_heatmap();
            }

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
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(200, realtime_poll_interval_ms_)));
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
    int thermometry_channel_{};
    int realtime_poll_interval_ms_{};
    bool heatmap_enable_{true};
    int heatmap_channel_{2};
    int heatmap_grid_rows_{2};
    int heatmap_grid_cols_{3};
    int active_thermometry_channel_{1};
    bool heatmap_dimensions_logged_{false};
    int last_heatmap_jpeg_width_{0};
    int last_heatmap_jpeg_height_{0};
    int last_heatmap_frame_width_{0};
    int last_heatmap_frame_height_{0};
    std::string last_heatmap_source_format_;

    std::atomic<bool> monitoring_active_{false};
    std::thread worker_;
    std::mutex thread_mutex_;
    std::mutex status_mutex_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr heatmap_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_alarm_srv_;

    std::vector<diagnostic_msgs::msg::KeyValue> latest_values_;
    std::vector<std::string> latest_artifacts_;
    bool latest_alarm_active_{false};
    std::string latest_alarm_type_;
    std::string latest_alarm_detail_;
    uint8_t latest_status_level_{diagnostic_msgs::msg::DiagnosticStatus::OK};
    bool realtime_thermometry_valid_{false};
    float realtime_temperature_c_{0.0F};
    float realtime_max_temperature_c_{0.0F};
    float realtime_min_temperature_c_{0.0F};
    float realtime_avg_temperature_c_{0.0F};
    float realtime_temperature_diff_c_{0.0F};
    std::string realtime_rule_name_{"realtime_thermometry"};
    std::string realtime_error_;
    uint8_t realtime_rule_id_{0};
    uint8_t realtime_calib_type_{2};
    std::vector<float> region_avg_temperatures_c_;
    std::vector<float> region_max_temperatures_c_;
    std::vector<float> region_min_temperatures_c_;
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
