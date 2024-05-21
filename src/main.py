import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
 
# Below function will read video imgs
cap = cv2.VideoCapture(0) 

GPIO.setmode(GPIO.BCM)
 
# GPIO Pinleri
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# Örnek olarak yazıldı
IN1=20
IN2=21
IN3=11
IN4=22
ENA=16
ENB=12
servoPIN=1
 
# IN => INPUT, OUT => OUTPUT
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(servoPIN, GPIO.OUT)

GPIO.output(IN1,GPIO.LOW)
GPIO.output(IN2,GPIO.LOW)
GPIO.output(IN3,GPIO.LOW)
GPIO.output(IN4,GPIO.LOW)

pwm = GPIO.PWM(servoPIN, 50) 

p_1=GPIO.PWM(ENA,1000)
p_1.start(0)

p_2=GPIO.PWM(ENA,1000)
p_2.start(0)


color_detected = ""

def renk_algila():
    while True:
        read_ok, img = cap.read()
        img_bcp = img.copy()
    
        img = cv2.resize(img, (640, 480))
        # Make a copy to draw contour outline
        input_image_cpy = img.copy()
    
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        # define range of red color in HSV
        lower_red = np.array([160,100,20])
        upper_red = np.array([179,255,255])
        
        # define range of green color in HSV
        lower_green = np.array([40, 20, 50])
        upper_green = np.array([90, 255, 255])
        
        # define range of blue color in HSV
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
    
        # create a mask for red color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        # create a mask for green color
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        # create a mask for blue color
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    
        # find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find contours in the green mask
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find contours in the blue mask
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        # loop through the red contours and draw a rectangle around them
        for cnt in contours_red:
            contour_area = cv2.contourArea(cnt)
            if contour_area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0,0,255), 2)
                cv2.putText(img, 'Red', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                return color_detected == "KIRMIZI"
        # loop through the blue contours and draw a rectangle around them
        for cnt in contours_blue:
            contour_area = cv2.contourArea(cnt)
            if contour_area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img, 'Blue', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                return color_detected == "YESIL"
    
        cv2.imshow('Color Recognition Output', img)
        
        # Close video window by pressing 'x'
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break
        else:
            gecen_sure +=1
            print(gecen_sure)

# Ön ultrasonik mesafe hesaplama ve uygulama
def front_distance():
    GPIO.output(GPIO_TRIGGER, True)
 
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    startTime = time.time()
    arrivalTime = time.time()
 
    while True:
        # Başlama Süresi
        while GPIO.input(GPIO_ECHO) == 0:
            startTime = time.time()
    
        # Dönüş Süresi
        while GPIO.input(GPIO_ECHO) == 1:
            arrivalTime = time.time()
    
        # Zaman farkı
        timeElapsed = arrivalTime - startTime
        front_distance = (timeElapsed * 34300) / 2
    
        return front_distance # cm cinsinden
        
front_distance = front_distance() # fonkisyonu değişkene atadım

#---------------------------------------------------------------------------------------#

# Sol ultrasonik mesafe hesaplama ve uygulama
def left_distance():
    GPIO.output(GPIO_TRIGGER, True)
 
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    startTime_2 = time.time()
    arrivalTime_2 = time.time()
 
    while True:
        # Başlama Süresi
        while GPIO.input(GPIO_ECHO) == 0:
            startTime_2 = time.time()
    
        # Dönüş Süresi
        while GPIO.input(GPIO_ECHO) == 1:
            arrivalTime_2 = time.time()
    
        # Zaman farkı
        timeElapsed_2 = arrivalTime_2 - startTime_2
        left_distance = (timeElapsed_2 * 34300) / 2
    
        return left_distance # cm cinsinden
 
left_distance = left_distance() # fonkisyonu değişkene atadım

#---------------------------------------------------------------------------------------#

# Sağ ultrasonik mesafe hesaplama ve uygulama
def right_distance():
    GPIO.output(GPIO_TRIGGER, True)
 
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    startTime_3 = time.time()
    arrivalTime_3 = time.time()
 
    while True:
        # Başlama Süresi
        while GPIO.input(GPIO_ECHO) == 0:
            startTime_3 = time.time()
    
        # Dönüş Süresi
        while GPIO.input(GPIO_ECHO) == 1:
            arrivalTime_3 = time.time()
    
        # Zaman farkı
        timeElapsed_3 = arrivalTime_3 - startTime_3
        right_distance = (timeElapsed_3 * 34300) / 2
    
        return right_distance # cm cinsinden
 
right_distance = right_distance() # fonkisyonu değişkene atadım

hiz = 100
color = renk_algila()
hareket_alg = 0 # kaçıncı adımda olduğu

# SERVO İÇİN:
# 12 = 180 Derece
# 1 = 15 Derece

def hareket_etme(hiz):
    # Loop içerisinde dönecek kod
    while True:
        # Yakınında engel bulunmuyorsa hızla ilerle
        while (front_distance > 10) and (left_distance > 10) and (right_distance > 10):
            GPIO.output(IN1,GPIO.HIGH)
            GPIO.output(IN2,GPIO.LOW)
            GPIO.output(IN3,GPIO.HIGH)
            GPIO.output(IN4,GPIO.LOW)
            p_1.ChangeDutyCycle(hiz)
            p_2.ChangeDutyCycle(hiz)
            
        while (front_distance <= 10) or (left_distance <= 10) or (right_distance <= 10):
            # Engel tespit edildiğinde
            if (color == "KIRMIZI"):
                hareket_alg = 1
                # Kırmızı renk tespit edildiğinde (sağından dönecek)
                # Servolar yardımıyla direksiyon kırılcak
                # Ardından güç verilecek

                # Sağa doğru ilerledi
                while(hareket_alg == 1):
                    pwm.ChangeDutyCycle(12) # Servo ayarlandı

                    # ilerledi 6cm kalıncaya kadar
                    while (left_distance > 6):
                        GPIO.output(IN1,GPIO.HIGH)
                        GPIO.output(IN2,GPIO.LOW)
                        GPIO.output(IN3,GPIO.HIGH)
                        GPIO.output(IN4,GPIO.LOW)
                        p_1.ChangeDutyCycle(hiz)
                        p_2.ChangeDutyCycle(hiz)

                    hareket_alg = 2
                    break
                # Sola doğru ilerledi
                while(hareket_alg == 2):
                    pwm.ChangeDutyCycle(0) # Servo ayarlandı 

                    while (right_distance > 6):
                        GPIO.output(IN1,GPIO.HIGH)
                        GPIO.output(IN2,GPIO.LOW)
                        GPIO.output(IN3,GPIO.HIGH)
                        GPIO.output(IN4,GPIO.LOW)
                        p_1.ChangeDutyCycle(hiz)
                        p_2.ChangeDutyCycle(hiz)

                    hareket_alg = 3
                    break
                # Direksiyonu düzeltip ilerledi
                while(hareket_alg==3):
                    pwm.ChangeDutyCycle(12) # Servo ayarlandı

                    GPIO.output(IN1,GPIO.HIGH)
                    GPIO.output(IN2,GPIO.LOW)
                    GPIO.output(IN3,GPIO.HIGH)
                    GPIO.output(IN4,GPIO.LOW)
                    p_1.ChangeDutyCycle(hiz)
                    p_2.ChangeDutyCycle(hiz)
                    time.sleep(3)

                    hareket_alg = 0
                    break

            elif (color=="YESIL"):
                hareket_alg = 4
                # Yeşil renk tespit edildiğinde (solundan dönecek)
                # Servolar yardımıyla direksiyon kırılcak
                # Ardından güç verilecek

                # Sola doğru ilerledi
                while(hareket_alg == 4):
                    pwm.ChangeDutyCycle(0) # Servo ayarlandı

                    while (right_distance > 6):
                        GPIO.output(IN1,GPIO.HIGH)
                        GPIO.output(IN2,GPIO.LOW)
                        GPIO.output(IN3,GPIO.HIGH)
                        GPIO.output(IN4,GPIO.LOW)
                        p_1.ChangeDutyCycle(hiz)
                        p_2.ChangeDutyCycle(hiz)

                    hareket_alg = 5
                    break
                # Sağa doğru ilerledi
                while(hareket_alg == 5):
                    pwm.ChangeDutyCycle(12) # Servo ayarlandı

                    while (left_distance > 6):
                        GPIO.output(IN1,GPIO.HIGH)
                        GPIO.output(IN2,GPIO.LOW)
                        GPIO.output(IN3,GPIO.HIGH)
                        GPIO.output(IN4,GPIO.LOW)
                        p_1.ChangeDutyCycle(hiz)
                        p_2.ChangeDutyCycle(hiz)

                    hareket_alg = 6
                    break
                # Direksiyonu düzeltip ilerledi
                while(hareket_alg==6):
                    pwm.ChangeDutyCycle(0) # Servo ayarlandı

                    GPIO.output(IN1,GPIO.HIGH)
                    GPIO.output(IN2,GPIO.LOW)
                    GPIO.output(IN3,GPIO.HIGH)
                    GPIO.output(IN4,GPIO.LOW)
                    p_1.ChangeDutyCycle(hiz)
                    p_2.ChangeDutyCycle(hiz)
                    time.sleep(3)

                    hareket_alg = 0
                    break

while True:
    hareket_etme(hiz)