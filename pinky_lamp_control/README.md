#pinky_lamp_control

## Prerequisite
Reference: https://github.com/jgarff/rpi_ws281x/wiki/Raspberry-Pi-5-Support

WS2812B is connected on Pin 19

Install `pinctrl` by https://github.com/raspberrypi/utils


## Shell script for Bootup Serivce

```shell
$ vi /home/robot/pinky_devices/lamp_module_bringup

#!/bin/bash

echo "load kernel module"
/usr/sbin/insmod /home/robot/pinky_devices/rpi_ws281x/rp1_ws281x_pwm/rp1_ws281x_pwm.ko pwm_channel=3
/usr/bin/dtoverlay -d /home/robot/pinky_devices/rpi_ws281x/rp1_ws281x_pwm rp1_ws281x_pwm
pinctrl set 19 a3 pn

while true; do
        sleep 5
done


$ chmod +x lamp_module_bringup
```

## Systemd Service
```shell
$ sudo systemctl --full --force edit lamp_bringup.service

[Unit]
Requires=ufw.service
After=ufw.service


[Service]
Type=simple
User=root
ExecStart=/home/robot/pinky_devices/lamp_module_bringup

[Install]
WantedBy=multi-user.target

$ sudo systemctl enable lamp_bringup.service
```
