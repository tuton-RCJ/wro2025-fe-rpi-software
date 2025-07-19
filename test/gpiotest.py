
import time
import serial

import RPi.GPIO as GPIO

BUZZER_PIN = 18  # buzzerのGPIOピン番号
LED_PIN = 17  # LEDのGPIOピン番号
SW_PIN = 19  # スイッチのGPIOピン番号

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # スイッチはプルアップ抵抗を使用

uart= serial.Serial('/dev/ttyAMA4', 115200)  # UARTの設定

pwm = GPIO.PWM(BUZZER_PIN, 440)  # 初期周波数440Hz（ラの音）

try:
    while True:
        if GPIO.input(SW_PIN) == GPIO.LOW :  # スイッチが押されたとき
            # ブザーを鳴らす
            pwm.start(50)  # デューティ比50%でスタート
            GPIO.output(LED_PIN, 1)
        else:
            # スイッチが離されたときはブザーを止める
            GPIO.output(LED_PIN, 0)
            pwm.stop()
        
        if uart.in_waiting > 0:  # UARTからのデータがある場合
            data = uart.read(uart.in_waiting)
            print(data)
            uart.write(data)
            
except KeyboardInterrupt:
    GPIO.cleanup()  # Ctrl+Cで終了したときにGPIOをクリーンアップ
finally:
    pwm.stop()
    GPIO.cleanup()  # プログラム終了時にGPIOをクリーンアップ
    uart.close()
    