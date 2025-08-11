import os
import sys
import spidev
import time
import RPi.GPIO as GPIO
import numpy as np
from PIL import ImageDraw, Image, ImageFont

# define the key button pin in BCM number
CS_PIN   = 8 
RST_PIN  = 1
DC_PIN   = 7
BL_PIN   = 12

class LCD():
    def __init__(self):
        try:
            # GPIO 초기화
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            GPIO.setup(CS_PIN, GPIO.OUT)    # CS 핀 출력 모드
            GPIO.setup(DC_PIN, GPIO.OUT)    # DC 핀 출력 모드
            GPIO.setup(RST_PIN, GPIO.OUT)   # RST 핀 출력 모드
            GPIO.setup(BL_PIN, GPIO.OUT)    # 백라이트 핀 출력 모드
            
            # 백라이트 초기값 설정
            self.bl = GPIO.PWM(BL_PIN, 1000)  # PWM 설정
            self.bl.start(90)  # 90% 백라이트 밝기

            # SPI 초기화
            self.bus = 0
            self.dev = 0
            self.spi_speed = 32000000
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.dev)
            self.spi.max_speed_hz = self.spi_speed
            self.spi.mode = 0b00

            # LCD 너비와 높이 설정
            self.w = 240
            self.h = 320
        except:
            print("현재 LCD가 사용중입니다. 사용 중인 커널을 종료 후 다시 실행해 주세요")
        
        self.lcd_init()
        self.clear()
        
    def _write_cmd(self, cmd):
        """write command"""
        GPIO.output(CS_PIN, GPIO.LOW)  # CS 활성화
        GPIO.output(DC_PIN, GPIO.LOW)  # 명령 모드
        self.spi.writebytes([cmd])
        GPIO.output(CS_PIN, GPIO.HIGH)  # CS 비활성화
        
    def _write_data(self, value):
        """write data"""
        GPIO.output(CS_PIN, GPIO.LOW)  # CS 활성화
        GPIO.output(DC_PIN, GPIO.HIGH)  # 데이터 모드
        self.spi.writebytes([value])
        GPIO.output(CS_PIN, GPIO.HIGH)  # CS 비활성화
        
    def reset(self):
        """reset the lcd"""
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(RST_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.01)

    def lcd_init(self):
        """lcd init..."""
        self.reset()
        
        self._write_cmd(0x36)
        self._write_data(0x00)
        
        self._write_cmd(0x3A) # set interface pixel format
        self._write_data(0x05)

        self._write_cmd(0x21)
        
        self._write_cmd(0x2A) # set x address form 0 to 320
        self._write_data(0x00)
        self._write_data(0x00)
        self._write_data(0x01) 
        self._write_data(0x3F)
        
        self._write_cmd(0x2B)  # set y address form 0 to 240
        self._write_data(0x00)
        self._write_data(0x00)
        self._write_data(0x00)
        self._write_data(0xEF)
        
        self._write_cmd(0xB2) # porch control
        self._write_data(0x0C)
        self._write_data(0x0C)
        self._write_data(0x00)
        self._write_data(0x33)
        self._write_data(0x33)

        self._write_cmd(0xB7) # gate control 
        self._write_data(0x35)
        
        self._write_cmd(0xBB)  # VCOMS setting
        self._write_data(0x1F)

        self._write_cmd(0xC0) #LCM control
        self._write_data(0x2C)
        
        self._write_cmd(0xC2)
        self._write_data(0x01)
        
        self._write_cmd(0xC3)
        self._write_data(0x12) 

        self._write_cmd(0xC4) # VDV setting
        self._write_data(0x20)

        self._write_cmd(0xC6)  # FR control 2
        self._write_data(0x0F)
        
        self._write_cmd(0xD0) # power control 1
        self._write_data(0xA4)
        self._write_data(0xA1)
        
        self._write_cmd(0xE0) # positive voltage gamma control
        self._write_data(0xD0)
        self._write_data(0x08)
        self._write_data(0x11)
        self._write_data(0x08)
        self._write_data(0x0C)
        self._write_data(0x15)
        self._write_data(0x39)
        self._write_data(0x33)
        self._write_data(0x50)
        self._write_data(0x36)
        self._write_data(0x13)
        self._write_data(0x14)
        self._write_data(0x29)
        self._write_data(0x2D)
        
        self._write_cmd(0xE1) # negative voltage gamma control 
        self._write_data(0xD0)
        self._write_data(0x08)
        self._write_data(0x10)
        self._write_data(0x08)
        self._write_data(0x06)
        self._write_data(0x06)
        self._write_data(0x39)
        self._write_data(0x44)
        self._write_data(0x51)
        self._write_data(0x0B)
        self._write_data(0x16)
        self._write_data(0x14)
        self._write_data(0x2F)
        self._write_data(0x31)
        self._write_cmd(0x21)
        self._write_cmd(0x11)
        self._write_cmd(0x29)
        
    def _set_windows(self, start_x, start_y, end_x, end_y):
        """display the windows of start point and end point"""
        self._write_cmd(0x2A)
        self._write_data(start_x >> 8)
        self._write_data(start_x & 0xff)
        self._write_data(end_x >> 8)
        temp = end_x - 1
        self._write_data(temp & 0xff)
        
        self._write_cmd(0x2B)
        self._write_data(start_y >> 8)
        self._write_data(start_y & 0xff)
        self._write_data(end_y >> 8)
        temp = end_y - 1
        self._write_data(temp & 0xff)
        
        self._write_cmd(0x2C)

    def set_backlight(self, value):
        self.bl.ChangeDutyCycle(value)

    def img_show(self, img):
        img = img.rotate(180)
        img = img.resize((self.h, self.w), Image.LANCZOS)
        img_w, img_h = img.size
        
        image = np.asarray(img.convert('RGB'))
        pixel = np.zeros((self.w, self.h, 2), dtype = np.uint8)
        pixel[..., [0]] = np.add(np.bitwise_and(image[..., [0]], 0xf8), np.right_shift(image[..., [1]], 5))
        
        pixel[..., [1]] = np.add(np.bitwise_and(np.left_shift(image[..., [1]], 3), 0xe0), np.right_shift(image[..., [2]], 3))       
        
        pixel = pixel.flatten().tolist()
        
        self._write_cmd(0x36)
        self._write_data(0x70)
        self._set_windows(0, 0, self.h, self.w)
        
        GPIO.output(DC_PIN, GPIO.HIGH)
        GPIO.output(CS_PIN, GPIO.LOW)
        for i in range(0, len(pixel), 4096):
            self.spi.writebytes(pixel[i:i+4096])
        GPIO.output(CS_PIN, GPIO.HIGH)

    def clear(self):
        self._set_windows(0, 0, self.w, self.h)
        buf = [0x00, 0x00] * (self.w * self.h)
        GPIO.output(DC_PIN, GPIO.HIGH)
        GPIO.output(CS_PIN, GPIO.LOW)
        for i in range(0, len(buf), 4096):
            self.spi.writebytes(buf[i:i+4096])
        GPIO.output(CS_PIN, GPIO.HIGH)

        self._set_windows(0, 0, self.h, self.w)
        buf = [0x00, 0x00] * (self.w * self.h)
        GPIO.output(DC_PIN, GPIO.HIGH)
        GPIO.output(CS_PIN, GPIO.LOW)
        for i in range(0, len(buf), 4096):
            self.spi.writebytes(buf[i:i+4096])
        GPIO.output(CS_PIN, GPIO.HIGH)

    def close(self):
        self.spi.close()
    
        GPIO.cleanup([CS_PIN, RST_PIN, DC_PIN, BL_PIN])
