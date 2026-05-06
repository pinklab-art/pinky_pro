# Pinky Pro
ROS2 packages for Pinky Pro

![Pinky Pro](doc/pinky_pro_image.png)
---
### Pinky Pro ROS2 아키텍처

![Pinky Pro ROS2 아키텍처](doc/architecture.png)

## 📚 문서 및 가이드 (Documentation)
* **[Pinky Pro 소개](https://github.com/pinklab-art/pinky_study/wiki)**

* **[초기 설정](https://github.com/pinklab-art/pinky_study/wiki/0.-%EC%B4%88%EA%B8%B0%EC%84%A4%EC%A0%95(PinkyPro))**

* **[수업 자료 (Part 1)](https://github.com/pinklab-art/pinky_study/wiki/2.1-Pinky-Pro(part1))**

* **[수업 자료 (Part 2)](https://github.com/pinklab-art/pinky_study/wiki/2.2-Pinky-Pro(part2))**

* **[수업 자료 (Part 3)](https://github.com/pinklab-art/pinky_study/wiki/2.3-Pinky-Pro(part3%E2%80%90%EC%8B%9C%EB%AE%AC%EB%A0%88%EC%9D%B4%EC%85%98))**

* **[수업 자료 (Part 4)](https://github.com/pinklab-art/pinky_study/wiki/2.4-Pinky-Pro(part4%E2%80%90%EC%8B%A4%EB%AC%BC%EB%A1%9C%EB%B4%87%ED%99%9C%EC%9A%A9))**
   

## 🙏 Special Thanks · Contributors

**[byeongkyu](https://github.com/byeongkyu)** – Pinky PRO 모델 ROS 2 패키지 개발  
참고 레포지토리: [pinky_robot](https://github.com/byeongkyu/pinky_robot)

# 💻 PC 설정

## 환경
* **OS:** Ubuntu 24.04
* **ROS:** ROS2 Jazzy
* **Architecture:** x86\_64 (amd64) (Recommended)
    * (ARM64 환경의 경우 [관련 문서](doc/arm64_guide.md)를 참고하세요.)

---

## 1. Pinky Pro ROS2 pkg clone

```
mkdir -p ~/pinky_pro/src
cd ~/pinky_pro/src
git clone https://github.com/pinklab-art/pinky_pro.git
```
## 2. dependency 설치
```
cd ~/pinky_pro
rosdep install --from-paths src --ignore-src -r -y
```
## 3. build
```
cd ~/pinky_pro
colcon build
```

# Pinky Pro 사용 매뉴얼

## 환경
- ubuntu 24.04
- ros2 jazzy

## pinky Pro 실행
```
ros2 launch pinky_bringup bringup_robot.launch.xml
```

## Map building
#### launch slam toolbox
```
ros2 launch pinky_navigation map_building.launch.xml
```
#### [ONLY PC] map view 
```
ros2 launch pinky_navigation map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name>
```

#### [ONLY PC] nav2 view
```
ros2 launch pinky_navigation nav2_view.launch.xml
```

# 시뮬레이션
## pinky Pro gazebo 실행
#### 가제보 실행
```
ros2 launch pinky_gz_sim launch_sim.launch.xml
```

## Map building
#### launch slam toolbox
```
ros2 launch pinky_navigation gz_map_building.launch.xml
```
#### [ONLY PC] map view 
```
ros2 launch pinky_navigation gz_map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch pinky_navigation gz_bringup_launch.xml map:=<map name>
```

#### [ONLY PC] nav2 view
```
ros2 launch pinky_navigation gz_nav2_view.launch.xml
```

# 센서 동작
## LED control
### LED server start
```
ros2 run pinky_led led_server
```
### LED service call
#### fill with color
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 255, g: 0, b: 0}"
```
#### set pixel colors
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'set_pixel', pixels: [4, 5, 6, 7], r: 0, g: 0, b: 255}"
```
#### clear
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'clear'}"
```
#### set brightness
```
ros2 service call /set_brightness pinky_interfaces/srv/SetBrightness "{brightness: 10}"
```
## LCD control
### emotion server start
```
ros2 run pinky_emotion emotion_server
```
or
```
ros2 run pinky_emotion emotion_server --ros-args -p load_frame_skip:=3
```

### set emotion
Available emotions: (hello, basic, angry, bored, fun, happy, interest, sad)
```
ros2 service call /set_emotion pinky_interfaces/srv/Emotion "{emotion: 'happy'}"
```

# 웹 서버
## Start web server
### Real Robot
Launch SLAM web server
```
ros2 launch pinky_navigation web_slam.launch.xml
```

Or launch Nav2 web server
```
ros2 launch pinky_navigation web_nav2.launch.xml map:=<map name>
```

With custom IP address (Optional)
```
ros2 launch pinky_navigation web_nav2.launch.xml map:=<map name> ip:=0.0.0.0
```
### Simulation (Gazebo)
Launch SLAM web server
```
ros2 launch pinky_navigation gz_web_slam.launch.xml
```

Or launch Nav2 web server
```
ros2 launch pinky_navigation gz_web_nav2.launch.xml map:=<map name>
```

With custom IP address (Optional)
```
ros2 launch pinky_navigation gz_web_nav2.launch.xml map:=<map name> ip:=0.0.0.0
```

## Web Access
**Default (Real Robot):**
[`http://192.168.4.1:8080`](http://192.168.4.1:8080)

**Default (Simulation):**
[`http://localhost:8080`](http://localhost:8080)

**If you specified a custom IP:**
`http://<host_ip>:<port>`
