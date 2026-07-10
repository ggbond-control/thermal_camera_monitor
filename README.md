# 热成像相机

## 编译

```bash
cd ~/Workspace/task_ws
colcon build --packages-select thermal_camera_monitor --symlink-install
```

## 启动

```bash
source ~/Workspace/task_ws/install/setup.zsh
ros2 launch thermal_camera_monitor thermal_camera_monitor.launch.py

ros2 service call /monitor/thermal_camera/start std_srvs/srv/Trigger "{}"
ros2 topic echo /monitor/thermal_camera/status | grep -E "\b(level|name|message|hardware_id)\b"
ros2 service call /monitor/thermal_camera/stop std_srvs/srv/Trigger "{}"
```

## 测试

```bash
ros2 service call /monitor/thermal_camera/test_alarm std_srvs/srv/Trigger "{}"
```

## 接口

| 名称                                 | 类型                                   |
| ------------------------------------ | -------------------------------------- |
| `/monitor/thermal_camera/start`      | `std_srvs/srv/Trigger`                 |
| `/monitor/thermal_camera/stop`       | `std_srvs/srv/Trigger`                 |
| `/monitor/thermal_camera/status`     | `diagnostic_msgs/msg/DiagnosticStatus` |
| `/monitor/thermal_camera/test_alarm` | `std_srvs/srv/Trigger`                 |
| `/monitor/thermal_camera/heatmap`    | `sensor_msgs/msg/Image`                |

`/monitor/thermal_camera/status`示例一：

```text
level: "\0"
name: thermal_camera
message: 热成像相机运行中
hardware_id: 192.168.2.64:8000
values:
  - {key: device_type, value: thermal_camera}
  - {key: device_ip, value: 192.168.2.64}
  - {key: device_port, value: '8000'}
  - {key: login_mode, value: '0'}
  - {key: save_pic_dir, value: /home/cat/Workspace/task_ws/src/thermal_camera_monitor/alarms}
  - {key: thermometry_channel, value: '1'}
  - {key: alarm_active, value: 'false'}
  - {key: alarm_type, value: ''}
  - {key: alarm_detail, value: ''}
  - {key: realtime_valid, value: 'true'}
  - {key: realtime_error, value: ''}
  - {key: temperature_c, value: '23.30'}
  - {key: max_temperature_c, value: '24.00'}
  - {key: min_temperature_c, value: '22.60'}
  - {key: avg_temperature_c, value: '23.30'}
  - {key: temperature_diff_c, value: '1.40'}
  - {key: realtime_rule_name, value: realtime_thermometry}
  - {key: realtime_rule_id, value: '0'}
  - {key: realtime_calib_type, value: '2'}
  - {key: region_temperature_rows, value: '2'}
  - {key: region_temperature_cols, value: '3'}
  - {key: region_temperature_count, value: '6'}
  - {key: region_avg_temperature_c, value: 23.35,23.44,23.54,23.29,23.36,23.45}
  - {key: region_max_temperature_c, value: 23.94,23.98,24.02,23.81,23.89,23.94}
  - {key: region_min_temperature_c, value: 22.79,22.79,23.00,22.75,22.79,22.96}
  - {key: artifact_count, value: '0'}
```

`/monitor/thermal_camera/status`示例二：

```text
level: "\x02"
name: thermal_camera
message: 热成像报警
hardware_id: 192.168.2.64:8000
values:
  - {key: device_type, : thermal_camera}
  - {key: device_ip, : 192.168.2.64}
  - {key: device_port, : '8000'}
  - {key: login_mode, : '0'}
  - {key: save_pic_dir, : /home/cat/Workspace/task_ws/src/thermal_camera_monitor/alarms}
  - {key: alarm_active, : 'true'}
  - {key: alarm_type, : thermal_alarm}
  - {key: alarm_detail, : 测温报警}
  - {key: artifact_count, : '2'}
  - {key: artifact_path_0, : /home/cat/Workspace/task_ws/src/thermal_camera_monitor/alarms/visible_20260604_164410.jpg}
  - {key: artifact_path_1, : /home/cat/Workspace/task_ws/src/thermal_camera_monitor/alarms/thermal_20260604_164410.jpg}
  - {key: command, : '21010'}
  - {key: temperature_c, : '38.10'}
  - {key: alarm_level, : '1'}
  - {key: alarm_type_code, : '0'}
  - {key: alarm_type_name, : 最高温}
  - {key: alarm_rule_code, : '0'}
  - {key: alarm_rule_name, : 大于}
  - {key: rule_temperature_c, : '35.00'}
  - {key: alarm_rule_temperature_c, : '35.00'}
```

## 排错

[192.168.2.64](http://192.168.2.64/)

`ssh -L 8080:192.168.2.64:80 cat@10.0.40.137`[127.0.0.1](http://localhost:8080)
