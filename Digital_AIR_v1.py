# code.py  CircuitPython 10.0.3
# Raspberry Pi Pico + TFT ST7789 320x240
# Features:
# - NTC 10k B3950 (3.3V--R10k--A1--NTC--GND)
# - PWM control to IBT_2 BTS7960
# - AUTO / MANUAL modes
# - Animated fan icon
# - Buttons: TEMP+/-, SPEED+/-
# - Show temperature, speed bar, voltage, © KT 2025


import microcontroller
import random


import time, math, board, busio, displayio, terminalio
from adafruit_st7789 import ST7789
from adafruit_display_text import label
from adafruit_display_shapes.rect import Rect
from adafruit_display_shapes.circle import Circle
from adafruit_display_shapes.line import Line
import digitalio, analogio, pwmio
from fourwire import FourWire

# ---------------------------
# Pin assignments
# ---------------------------
TFT_CS = board.GP11
TFT_DC = board.GP12
#SPI_CLK = board.GP14
#SPI_MOSI = board.GP15
#BACKLIGHT_PIN = board.GP20

# Buttons
BTN_TEMP_PLUS = board.GP2
BTN_TEMP_MINUS = board.GP3
BTN_SPEED_PLUS = board.GP4
BTN_SPEED_MINUS = board.GP5

# ADCs
ADC_NTC = board.A1
ADC_VOLT = board.A2

# Motor PWM (IBT_2)
PWM_MOTOR = board.GP17

# ---------------------------
# Constants
# ---------------------------
R_FIXED = 10000.0
NTC_R0 = 10000.0
NTC_BETA = 3950.0
T0_K = 25 + 273.15

TEMP_MIN, TEMP_MAX = 5, 50
SPEED_MIN, SPEED_MAX = 1, 10
BATTERY_VOLTAGE_DIVIDER = 4.0

PWM_FREQ = 2000
SPEED_SMOOTHING = 0.15
BACKLIGHT_BRIGHTNESS = 0.6
UPDATE_INTERVAL = 0.3

# ---------------------------
# Setup hardware
# ---------------------------

# --- ตั้งค่าจอ ST7789 ---
displayio.release_displays()
spi = busio.SPI(board.GP14, board.GP15)
tft_dc = board.GP12
tft_cs = board.GP11
tft_reset = None

display_bus = FourWire(spi, command=tft_dc, chip_select=tft_cs, reset=tft_reset)

display = ST7789(
    display_bus,
    width=320,
    height=240,
    rotation=90,     # หมุนให้แนวนอน ถ้าจอแสดงกลับด้านให้ลอง 90 หรือ 0
    rowstart=0,       # ป้องกันบางรุ่นที่ crop ภาพ
    colstart=0
)

# --- Backlight ---
backlight = digitalio.DigitalInOut(board.GP20)
backlight.direction = digitalio.Direction.OUTPUT
backlight.value = True

# Backlight
#bl = pwmio.PWMOut(BACKLIGHT_PIN, frequency=1000, duty_cycle=int(55000 * BACKLIGHT_BRIGHTNESS))

# ADC
adc_ntc = analogio.AnalogIn(ADC_NTC)
adc_v = analogio.AnalogIn(ADC_VOLT)

# Buttons
def make_button(pin):
    b = digitalio.DigitalInOut(pin)
    b.direction = digitalio.Direction.INPUT
    b.pull = digitalio.Pull.UP
    return b

btn_temp_plus = make_button(BTN_TEMP_PLUS)
btn_temp_minus = make_button(BTN_TEMP_MINUS)
btn_speed_plus = make_button(BTN_SPEED_PLUS)
btn_speed_minus = make_button(BTN_SPEED_MINUS)

# Motor PWM
motor_pwm = pwmio.PWMOut(PWM_MOTOR, frequency=PWM_FREQ, duty_cycle=0)

# ---------------------------
# Helper functions
# ---------------------------
def adc_to_voltage(adc):
    return (adc.value / 65535) * 3.3

def read_ntc_temp():
    v = adc_to_voltage(adc_ntc)
    if v <= 0 or v >= 3.3:
        return 25
    r_ntc = R_FIXED * (v / (3.3 - v))
    temp_k = 1 / ((1 / T0_K) + (1 / NTC_BETA) * math.log(r_ntc / NTC_R0))
    return temp_k - 273.15

def read_voltage():
    return adc_to_voltage(adc_v) * BATTERY_VOLTAGE_DIVIDER

def temp_to_speed(t):
    t = max(min(t, TEMP_MAX), TEMP_MIN)
    frac = (t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)
    return int(SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN))

# ---------------------------
# Display setup
# ---------------------------
root = displayio.Group()
display.root_group = root
root.append(Rect(0, 0, 320, 240, fill=0x000000))

font = terminalio.FONT
temp_label = label.Label(font, text="--°C", scale=4, color=0xFFFFFF, x=160, y=100, anchor_point=(0.5,0.5))
speed_label = label.Label(font, text="Speed: --", scale=2, color=0x00FF00, x=160, y=170, anchor_point=(0.5,0.5))
volt_label = label.Label(font, text="Volt: --V", scale=1, color=0x00FFFF, x=6, y=6)
mode_label = label.Label(font, text="AUTO", scale=1, color=0xFFFFFF, x=260, y=6)
copyright_label = label.Label(font, text="© KT 2025", scale=1, color=0xAAAAAA, x=6, y=226)

root.append(temp_label)
root.append(speed_label)
root.append(volt_label)
root.append(mode_label)
root.append(copyright_label)

# Fan icon (animated)
fan_group = displayio.Group(x=40, y=120)
fan_circle = Circle(0, 0, 26, outline=0x00AAFF)
fan_group.append(fan_circle)
blade_lines = [
    Line(-20, 0, 20, 0, color=0x00AAFF),
    Line(0, -20, 0, 20, color=0x00AAFF),
    Line(-14, -14, 14, 14, color=0x00AAFF),
]
for b in blade_lines:
    fan_group.append(b)
root.append(fan_group)

# Speed bar
bars = []
for i in range(10):
    bar = Rect(200 + i*10, 210 - i*10, 8, i*10, fill=0x222222)
    root.append(bar)
    bars.append(bar)

# ---------------------------
# Control variables
# ---------------------------
mode_auto = True
set_temp = 25
manual_speed = 5
display_speed = 0
fan_angle = 0
last_update = 0

# ---------------------------
# Main loop
# ---------------------------
while True:
    t = read_ntc_temp()
    vbat = read_voltage()

    # Buttons
    if not btn_temp_plus.value:
        set_temp = min(set_temp + 1, 50)
        time.sleep(0.2)
    if not btn_temp_minus.value:
        set_temp = max(set_temp - 1, 5)
        time.sleep(0.2)

    if not btn_speed_plus.value:
        manual_speed += 1
        if manual_speed > SPEED_MAX:
            mode_auto = True
            manual_speed = SPEED_MAX
        else:
            mode_auto = False
        time.sleep(0.2)

    if not btn_speed_minus.value:
        manual_speed = max(manual_speed - 1, SPEED_MIN)
        mode_auto = False
        time.sleep(0.2)

    # Compute target speed
    if mode_auto:
        target_speed = temp_to_speed(t)
    else:
        target_speed = manual_speed

    display_speed = (1 - SPEED_SMOOTHING) * display_speed + SPEED_SMOOTHING * target_speed

    duty = int((display_speed / SPEED_MAX) * 65535)
    motor_pwm.duty_cycle = duty

    # Fan rotation animation
    fan_angle += display_speed * 10
    sin_a = math.sin(math.radians(fan_angle))
    cos_a = math.cos(math.radians(fan_angle))
    blade_coords = [(-20,0,20,0), (0,-20,0,20), (-14,-14,14,14)]
    for b, coords in zip(blade_lines, blade_coords):
        x1 = int(coords[0]*cos_a - coords[1]*sin_a)
        y1 = int(coords[0]*sin_a + coords[1]*cos_a)
        x2 = int(coords[2]*cos_a - coords[3]*sin_a)
        y2 = int(coords[2]*sin_a + coords[3]*cos_a)
        b.x1, b.y1, b.x2, b.y2 = x1, y1, x2, y2

    # Update display
    now = time.monotonic()
    if now - last_update > UPDATE_INTERVAL:
        temp_label.text = f"{t:4.1f}°C"
        speed_label.text = f"Speed: {int(display_speed)}"
        volt_label.text = f"Volt: {vbat:4.2f}V"
        mode_label.text = "AUTO" if mode_auto else "MANUAL"

        # bars update
        for i, bar in enumerate(bars, start=1):
            bar.fill = 0x00FF00 if i <= int(display_speed) else 0x222222

        last_update = now

    time.sleep(0.05)
