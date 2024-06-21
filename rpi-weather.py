import RPi.GPIO as GPIO
import time
from time import sleep
import adafruit_dht
import board
import neopixel
from gpiozero import Button
from gpiozero import PWMOutputDevice
import paho.mqtt.client as mqtt
import psutil


LED = 23
GPIO.setup(LED, GPIO.OUT)

buzzer_pin=12
music_on = True
tones = [493,391]
music = [1,2,1,2,1,2]
term = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
a = [1,2,3,4]
def play_music():
    global music, tones, term
    pwm_device = PWMOutputDevice(pin=buzzer_pin, frequency=100, initial_value=0.0)
   
    while True:
        for i in range(len(a)):
            pwm_device.frequency = tones[music[i] - 1]  
            pwm_device.value = 0.5  
            sleep(term[i])
            pwm_device.value = 0.0
        break

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pixel_pin = board.D10
num_pixels = 4
pixels = neopixel.NeoPixel(pixel_pin , num_pixels, brightness=0.8, auto_write=False, pixel_order=neopixel.GRB)

BUTTON = 24
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
dht_device = adafruit_dht.DHT22(board.D4)

debounce_time = 0.3
prev_button_time = 0
def is_button_pressed():
    global prev_button_time
    current_time = time.time()
    if GPIO.input(BUTTON) == GPIO.HIGH:
        if (current_time - prev_button_time) >= debounce_time:
            prev_button_time = current_time
            return True
    return False


MY_ID = "21"

def get_data():
    while True:
        try:
            temperature = dht_device.temperature
            humidity = dht_device.humidity
            return temperature, humidity
        except RuntimeError as error:
            sleep(2.0)
            continue

MQTT_HOST = "mqtt-dashboard.com" # broker
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 60
MQTT_TOPIC = f"weather/{MY_ID}/sensing"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
client.loop_start()


try:
    count=0
    while True:
        if is_button_pressed():
            count+=1
            try:
                temperature = dht_device.temperature
                humidity = dht_device.humidity
                print(temperature, humidity)

                sleep(1)
                temperature, humidity = get_data()

                if float(temperature) <= 25.4:
                    if float(humidity) <= 50:
                        pixels.fill((0,210,255))
                        pixels.show()
                        GPIO.output(LED, GPIO.HIGH)
                        sleep(2)
                        GPIO.output(LED, GPIO.LOW)
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 하늘색}}'
                    else:
                        pixels.fill((0,0,255))
                        pixels.show()
                        play_music()
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 파란색}}'


                if float(temperature) > 25.4 and float(temperature) <= 25.6:
                    if float(humidity) <= 50:
                        pixels.fill((255,150,0))
                        pixels.show()
                        GPIO.output(LED, GPIO.HIGH)
                        sleep(2)
                        GPIO.output(LED, GPIO.LOW)
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 노란색}}'
                    else:
                        pixels.fill((0,255,12))
                        pixels.show()
                        play_music()
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 초록색}}'

                if float(temperature) > 25.6:
                    if float(humidity) <= 50:
                        pixels.fill((250,50,250))
                        pixels.show()
                        GPIO.output(LED, GPIO.HIGH)
                        sleep(2)
                        GPIO.output(LED, GPIO.LOW)
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 분홍색}}'
                    else:
                        pixels.fill((255,0,0))
                        pixels.show()
                        play_music()
                        value = f'{{"현재온도": {temperature:.1f}, "현재습도": {humidity}, "눈으로 보는 날씨: 빨간색}}'

                client.publish(MQTT_TOPIC, value)

                if count > 5:
                    pixels.fill((0,0,0))
                    pixels.show()
                    print("작동이 끝났습니다.")
                    break

            except RuntimeError:
                sleep(1.0)            
except KeyboardInterrupt:
    pixels.fill((0,0,0))
    pixels.show()
    print("I'm done!")

finally:
    client.loop_stop()
    client.disconnect()
