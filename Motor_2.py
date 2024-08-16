import RPi.GPIO as GPIO
import time

in1 = 24
in2 = 23
en = 25
temp1 = 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
p = GPIO.PWM(en, 1000)

p.start(0)  # PWM sinyalini başlangıçta 0 olarak ayarla, motoru tamamen durdur

def run(command):
        if command == 'start':
                GPIO.output(in1, GPIO.HIGH)
                GPIO.output(in2, GPIO.LOW)
                en.ChangeDutyCycle(50)  # %50 hız
        elif command == 'stop':
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.LOW)
                en.ChangeDutyCycle(0)  # Motoru durdur
        elif command == 'reverse':
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.HIGH)
                en.ChangeDutyCycle(50)  # %50 hız
    

def motor_control(command):
        if command == 'D':
                run('start')
                time.sleep(5)
        elif command == 'B':
                run('stop')
                time.sleep(2)
        elif command == 'A':
                run('reverse')
                time.sleep(5)
        else:
                print("Lutfen bir tusa basiniz")
        
def cleanup():
        p.stop()
        GPIO.cleanup() 
