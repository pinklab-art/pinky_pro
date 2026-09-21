# ARM64 PC 환경 설정 가이드
현재 패키지를 arm64 환경에서 사용하려면 아래 과정이 필요합니다
## PC 설정

### 1. Pinky Pro ROS2 pkg clone
```
mkdir -p ~/pinky_pro/src
cd ~/pinky_pro/src
git clone https://github.com/pinklab-art/pinky_pro.git
```
### 2. (Gazebo 사용 시) pinky_gz_sim 패키지의 CMakeLists.txt 수정
8-11 번째 줄 부분 삭제하거나 주석 처리
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "This package is skipped on aarch64.")
  return()
endif()
```
### 2-1. (MuJoCo 사용 시) pinky_mujoco 패키지의 setup.py 수정
aarch64 건너뛰기 부분을 삭제하거나 주석 처리
```
if platform.machine() == 'aarch64':
    print(f'{package_name} is a PC simulation package. Skip building on aarch64.')
    setup(name=package_name, version='0.1.0', packages=[], data_files=[], zip_safe=True)
    raise SystemExit(0)
```
### 3. 하드웨어 센서 관련 패키지 삭제
```
cd ~/pinky_pro/src/pinky_pro
sudo rm -rf pinky_emotion pinky_imu_bno055 pinky_lamp_control pinky_led pinky_sensor_adc
```
### 4. 의존성 설치 (Dependency)
```
cd ~/pinky_pro
rosdep install --from-paths src --ignore-src -r -y
```
### 5. 빌드 (Build)
```
cd ~/pinky_pro
colcon build
```