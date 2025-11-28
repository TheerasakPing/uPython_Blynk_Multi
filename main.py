"""
PROJECT: IoT Smart Controller (Full Options: ADC + Dual Switch)
FILE: main.py
AUTHOR: Theera co-engineering AI Senior Architect
DATE: 2025-11-28
VERSION: 1.0.0

DESCRIPTION:
Firmware ควบคุมและอ่านค่า Sensor สำหรับ Production
1. ควบคุม LED 2 ดวง ผ่านปุ่ม V0 และ V1
2. อ่านค่า Analog (A0, A1) และส่งขึ้น Server (V5, V6) อัตโนมัติ
3. รองรับทั้ง ESP32 และ Raspberry Pi Pico W

HARDWARE MAPPING:
| Feature | Blynk Pin | ESP32 Pin | Pico W Pin |
| :--- | :--- | :--- | :--- |
| Switch 1 | V0 | GPIO 2 (Built-in) | 'LED' (Built-in) |
| Switch 2 | V1 | GPIO 4 | GP15 |
| Sensor 1 | V5 (A0) | GPIO 34 | GP26 (ADC0) |
| Sensor 2 | V6 (A1) | GPIO 35 | GP27 (ADC1) |

CHANGELOG:
- v1.0.0: Initial release with basic functionality. Removed sensitive credentials and added placeholders for user configuration.
"""

import machine
import network
import time
import gc
import sys
from machine import Pin, WDT, ADC
from micropython import const

# --- Configuration ---
# NOTE: Please configure your WiFi and Blynk credentials below.
# For security, consider loading these from a separate, untracked configuration file
# or environment variables in a production environment.
_WIFI_SSID = "YOUR_WIFI_SSID"  # Replace with your WiFi network name
_WIFI_PASS = "YOUR_WIFI_PASSWORD"  # Replace with your WiFi password
_BLYNK_AUTH = "YOUR_BLYNK_AUTH_TOKEN"  # Replace with your Blynk Auth Token

# ตั้งเวลาส่งข้อมูล Sensor (มิลลิวินาที) - ส่งทุก 2 วินาที
_SENSOR_INTERVAL = 2000

# --- Platform & Hardware Config ---
PLATFORM = sys.platform
print(f"[SYSTEM] Platform Detected: {PLATFORM}")

# กำหนดขา Hardware ตามรุ่นบอร์ด
if PLATFORM == "rp2":  # Raspberry Pi Pico W
    # Output Pins
    _PIN_LED_V0 = "LED"
    _PIN_LED_V1 = 15
    # Input ADC Pins
    _PIN_ADC_A0 = 26
    _PIN_ADC_A1 = 27
    # System Config
    _WDT_TIMEOUT = 8000

else:  # ESP32
    # Output Pins
    _PIN_LED_V0 = 2
    _PIN_LED_V1 = 4
    # Input ADC Pins (ใช้ขา Input Only เพื่อความแม่นยำ)
    _PIN_ADC_A0 = 34
    _PIN_ADC_A1 = 35
    # System Config
    _WDT_TIMEOUT = 20000

# --- Import Library ---
try:
    import BlynkLib
except ImportError:
    print("[CRITICAL] Missing 'BlynkLib.py'")
    raise SystemExit


class DeviceController:
    def __init__(self):
        self._init_hardware()
        self._init_network()

        self.blynk = None
        self._last_gc = time.ticks_ms()
        self._last_sensor_sent = 0  # ตัวจับเวลาส่งค่า Sensor

    def _init_hardware(self):
        """ตั้งค่า Pin และ ADC"""
        print("[HW] Initializing GPIO & ADC...")
        try:
            # 1. Setup LEDs
            self.led_v0 = Pin(_PIN_LED_V0, Pin.OUT)
            self.led_v1 = Pin(_PIN_LED_V1, Pin.OUT)
            self.led_v0.value(0)
            self.led_v1.value(0)

            # 2. Setup ADC (Analog Inputs)
            self.adc_a0 = ADC(Pin(_PIN_ADC_A0))
            self.adc_a1 = ADC(Pin(_PIN_ADC_A1))

            if PLATFORM != "rp2":  # Config สำหรับ ESP32
                # ปรับความกว้างสัญญาณเป็น 11dB (อ่านค่า 0-3.3V ได้เต็มช่วง)
                self.adc_a0.atten(ADC.ATTN_11DB)
                self.adc_a1.atten(ADC.ATTN_11DB)

            print(
                f"[HW] Success: LED1={_PIN_LED_V0}, LED2={_PIN_LED_V1}, A0={_PIN_ADC_A0}, A1={_PIN_ADC_A1}"
            )

        except Exception as e:
            print(f"[HW] Init Error: {e}")

    def _init_network(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

    def connect_wifi_blocking(self):
        """เชื่อมต่อ WiFi"""
        print(f"[WIFI] Connecting to {_WIFI_SSID}...")

        # Pico W Fix
        if PLATFORM == "rp2":
            self.wlan.config(pm=0xA11140)
            if self.wlan.isconnected():
                self.wlan.disconnect()
                time.sleep(1)

        if not self.wlan.isconnected():
            self.wlan.connect(_WIFI_SSID, _WIFI_PASS)
            for _ in range(30):
                if self.wlan.isconnected():
                    # Pico W Check
                    if PLATFORM == "rp2" and self.wlan.status() != 3:
                        pass
                    else:
                        break
                time.sleep(1)
                print(".", end="")

        if self.wlan.isconnected():
            print(f"\n[WIFI] Connected! IP: {self.wlan.ifconfig()[0]}")
        else:
            print("\n[CRITICAL] WiFi Failed! Resetting...")
            time.sleep(2)
            machine.reset()

    def init_blynk(self):
        print("[BLYNK] Connecting...")
        try:
            self.blynk = BlynkLib.Blynk(_BLYNK_AUTH, insecure=True, heartbeat=30)
            self._register_handlers()
            print("[BLYNK] Ready.")
        except Exception as e:
            print(f"[BLYNK] Error: {e}")
            time.sleep(2)
            machine.reset()

    def _register_handlers(self):
        if not self.blynk:
            return

        # --- Handler V0 (Switch 1) ---
        @self.blynk.on("V0")
        def v0_handler(value):
            try:
                cmd = int(value[0])
                print(f"[OP] V0 Switch -> {cmd}")
                self.led_v0.value(cmd)
            except:
                pass

        # --- Handler V1 (Switch 2) ---
        @self.blynk.on("V1")
        def v1_handler(value):
            try:
                cmd = int(value[0])
                print(f"[OP] V1 Switch -> {cmd}")
                self.led_v1.value(cmd)
            except:
                pass

        # --- On Connect ---
        @self.blynk.on("connected")
        def on_connect(ping=None):
            print(f"[STATUS] Online! Ping: {ping}ms")
            try:
                # Sync ปุ่มทั้ง 2 ให้ตรงกับแอป
                self.blynk.sync_virtual(0, 1)
            except:
                pass

    def read_and_send_sensors(self):
        """อ่านค่า ADC และส่งไปที่ Blynk V5, V6"""
        try:
            # อ่านค่า Raw (ESP32: 0-4095, Pico: 0-65535)
            val_a0 = (
                self.adc_a0.read_u16()
            )  # ใช้ read_u16 เพื่อให้ได้ค่าสเกลเดียวกันทั้งสองบอร์ด (0-65535)
            val_a1 = self.adc_a1.read_u16()

            # แปลงเป็น Voltage (คร่าวๆ 3.3V ref)
            volt_a0 = (val_a0 / 65535) * 3.3
            volt_a1 = (val_a1 / 65535) * 3.3

            # ส่งค่าขึ้น Server (Virtual Pin 5 และ 6)
            # เราส่งค่า Voltage ไปเพื่อให้ดูง่าย หรือจะส่ง val_a0 ดิบๆ ก็ได้
            self.blynk.virtual_write(5, convert_to_blynk_val(volt_a0))
            self.blynk.virtual_write(6, convert_to_blynk_val(volt_a1))

            print(
                f"[SENSOR] A0: {val_a0}({volt_a0:.2f}V) | A1: {val_a1}({volt_a1:.2f}V)"
            )

        except Exception as e:
            print(f"[ERR] Sensor Read: {e}")

    def run(self):
        print("\n=== THEERA CO-ENGINEERING AI (v1.0.0) ===")
        self.connect_wifi_blocking()
        self.init_blynk()

        wdt = WDT(timeout=_WDT_TIMEOUT)
        print(f"[SYSTEM] Running... (WDT: {_WDT_TIMEOUT}ms)")

        while True:
            wdt.feed()

            # 1. WiFi Guard
            if not self.wlan.isconnected():
                print("[WARN] WiFi Drop! Resetting...")
                time.sleep(1)
                machine.reset()

            # 2. Blynk Process
            try:
                self.blynk.run()
            except Exception:
                print("[ERR] Network Error. Resetting...")
                time.sleep(1)
                machine.reset()

            # 3. Sensor Loop (Non-blocking)
            now = time.ticks_ms()
            if time.ticks_diff(now, self._last_sensor_sent) > _SENSOR_INTERVAL:
                self.read_and_send_sensors()
                self._last_sensor_sent = now

            # 4. GC
            if time.ticks_diff(now, self._last_gc) > 10000:
                gc.collect()
                self._last_gc = now


def convert_to_blynk_val(num):
    """Helper formatting float to string"""
    return "{:.2f}".format(num)


if __name__ == "__main__":
    try:
        app = DeviceController()
        app.run()
    except KeyboardInterrupt:
        print("\n[User Stop]")
    except Exception as e:
        print(f"\n[Crash] {e}")
        time.sleep(5)
        machine.reset()
