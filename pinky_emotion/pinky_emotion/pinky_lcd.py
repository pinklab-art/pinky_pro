import spidev
import time
import RPi.GPIO as GPIO
import numpy as np
from PIL import Image

RST_PIN  = 27
DC_PIN   = 25
BL_PIN   = 18

class LCD():
    def __init__(self):
        # LCD 해상도
        self.w = 240
        self.h = 320
        
        # GPIO 초기화
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(RST_PIN, GPIO.OUT)
        GPIO.setup(DC_PIN, GPIO.OUT)
        GPIO.setup(BL_PIN, GPIO.OUT)
        
        # 백라이트 PWM 설정
        self.bl = GPIO.PWM(BL_PIN, 1000)
        self.bl.start(100)  # 백라이트 밝기 100%로 시작

        # SPI 초기화 
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) # bus=0, device=0 (CE0)
        self.spi.max_speed_hz = 80000000 # 40MHz
        self.spi.mode = 0b00

        self.lcd_init()
        
    def _write_cmd(self, cmd):
        GPIO.output(DC_PIN, GPIO.LOW)
        self.spi.writebytes([cmd])
        
    def _write_data(self, val):
        GPIO.output(DC_PIN, GPIO.HIGH)
        self.spi.writebytes([val])

    def _write_data_buffer(self, buf):
        GPIO.output(DC_PIN, GPIO.HIGH)
        self.spi.writebytes(buf)

    def reset(self):
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(RST_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RST_PIN, GPIO.HIGH)
        time.sleep(0.01)
        
    def lcd_init(self):
        self.reset()

        self._write_cmd(0x11) # Sleep out
        time.sleep(0.12)

        self._write_cmd(0xCF); self._write_data(0x00); self._write_data(0xC1); self._write_data(0X30)
        self._write_cmd(0xED); self._write_data(0x64); self._write_data(0x03); self._write_data(0X12); self._write_data(0X81)
        self._write_cmd(0xE8); self._write_data(0x85); self._write_data(0x00); self._write_data(0x79)
        self._write_cmd(0xCB); self._write_data(0x39); self._write_data(0x2C); self._write_data(0x00); self._write_data(0x34); self._write_data(0x02)
        self._write_cmd(0xF7); self._write_data(0x20)
        self._write_cmd(0xEA); self._write_data(0x00); self._write_data(0x00)
        
        self._write_cmd(0xC0); self._write_data(0x1D) # Power control
        self._write_cmd(0xC1); self._write_data(0x12) # Power control
        self._write_cmd(0xC5); self._write_data(0x33); self._write_data(0x3F) # VCM control
        self._write_cmd(0xC7); self._write_data(0x92) # VCM control
        
        self._write_cmd(0x3A); self._write_data(0x55) # Pixel Format Set (RGB565)
        self._write_cmd(0x36); self._write_data(0x08) # Memory Access Control
        
        self._write_cmd(0xB1); self._write_data(0x00); self._write_data(0x12)
        self._write_cmd(0xB6); self._write_data(0x0A); self._write_data(0xA2) # Display Function Control
        
        self._write_cmd(0x44); self._write_data(0x02)
        self._write_cmd(0xF2); self._write_data(0x00) # 3Gamma Function Disable
        self._write_cmd(0x26); self._write_data(0x01) # Gamma curve selected
        
        # Positive Gamma Correction
        self._write_cmd(0xE0)
        self._write_data_buffer([0x0F, 0x22, 0x1C, 0x1B, 0x08, 0x0F, 0x48, 0xB8, 0x34, 0x05, 0x0C, 0x09, 0x0F, 0x07, 0x00])
        
        # Negative Gamma Correction
        self._write_cmd(0xE1)
        self._write_data_buffer([0x00, 0x23, 0x24, 0x07, 0x10, 0x07, 0x38, 0x47, 0x4B, 0x0A, 0x13, 0x06, 0x30, 0x38, 0x0F])
        
        self._write_cmd(0x29) # Display on

    def _set_windows(self, start_x, start_y, end_x, end_y):
        self._write_cmd(0x2A) # Column Address Set
        self._write_data(start_x >> 8)
        self._write_data(start_x & 0xff)
        self._write_data((end_x - 1) >> 8)
        self._write_data((end_x - 1) & 0xff)

        self._write_cmd(0x2B) # Page Address Set
        self._write_data(start_y >> 8)
        self._write_data(start_y & 0xff)
        self._write_data((end_y - 1) >> 8)
        self._write_data((end_y - 1) & 0xff)

        self._write_cmd(0x2C) # Memory Write

    def img_show(self, img):
        img = img.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.ROTATE_270)
        img = img.resize((self.w, self.h), Image.LANCZOS)
        
        image = np.asarray(img.convert('RGB'))
        
        pixel = np.zeros((self.h, self.w, 2), dtype=np.uint8)
        pixel[..., [0]] = np.add(np.bitwise_and(image[..., [0]], 0xF8), np.right_shift(image[..., [1]], 5))
        pixel[..., [1]] = np.add(np.bitwise_and(np.left_shift(image[..., [1]], 3), 0xE0), np.right_shift(image[..., [2]], 3))
        
        pixel = pixel.flatten().tolist()
        
        self._write_cmd(0x36)
        self._write_data(0x08)
        self._set_windows(0, 0, self.w, self.h)
        
        for i in range(0, len(pixel), 4096):
            self._write_data_buffer(pixel[i:i+4096])

    def clear(self, color=0x0000):
        color_high = color >> 8
        color_low = color & 0xff
        
        buf = [color_high, color_low] * (self.w * self.h)
        
        self._set_windows(0, 0, self.w, self.h)
        for i in range(0, len(buf), 4096):
            self._write_data_buffer(buf[i:i+4096])
            
    def set_backlight(self, value):
        if value < 0: value = 0
        if value > 100: value = 100
        self.bl.ChangeDutyCycle(value)

    def close(self):
        self.spi.close()
        self.bl.stop()
        GPIO.cleanup([RST_PIN, DC_PIN, BL_PIN])