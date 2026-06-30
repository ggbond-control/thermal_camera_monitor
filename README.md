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
ros2 topic echo /monitor/thermal_camera/status | grep -E "(level|name|message|hardware_id)"
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

`/monitor/thermal_camera/status`示例：

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
