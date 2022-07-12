
# libraries required to enhance fps
# fps değerini arttırmak için gerekli olan kütüphaneler
import imutils
from imutils.video import WebcamVideoStream
from imutils.video import FPS


import cv2
import numpy as np
import math
import datetime
import RPi.GPIO as GPIO

# library required to send ball locations to remote ground station
# uzak mesafe haberleşme için gerekli olan kütüphane
from lib_nrf24 import NRF24


# spi protocol library
# spi kütüphanesi
import spidev


import time

# library required for pwm signal generation
# pwm sinyalini üretmek için gerekli kütüphane
import pigpio

# libraries required to draw the graph of ball location changes end of the processes
# işlemler sonunda top konum değişimlerinin grafiğini çizdirmek için gerekli kütüphaneler
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# gpio setting
# gpio ayarlama
GPIO.setmode(GPIO.BCM)


# producing video capture object
# video yakalama nesnesini oluşturma
stream = cv2.VideoCapture(0)



# arranging hough transform radius and threshold for center detection
# hough dönüşümünde merkez tespiti için eşik değeri ve yarıçap ayarlaması
table_center_min_radius = math.floor(6*300/480) # 300x300 istenen pencere, 480x480 alınan pencere
table_center_max_radius =  math.floor(9*300/480)
table_center_param2 = math.ceil(20*300/480)

ball_center_min_radius = math.floor(8*300/480)
ball_center_max_radius =  math.floor(40*300/480)
ball_center_param2 = math.ceil(40*300/480)

# distance calculation between found circles' centers
# çemberler arasında merkez mesafesi hesaplama
dist = lambda x1, y1, x2, y2: (x1-x2)**2 + (y1 - y2)**2




# pid variables
# pid değişkenler
prev_time = None
x_prev_error = 0
y_prev_error = 0
x_error_sum = 0
y_error_sum = 0

dt_sum = 0
# this function is to calculate servo degrees from ball's x and y location
# topun x ve y konumundan servo açılarını hesaplamak için kullanılan fonksiyon
# x_input: x location
# y_input: y location
# x_set_point: x location set point
# y_set_point: y location set point
# kp: proportional gain
# ki: integral gain
# kd: derivative gain
# max_out: servo max output degree

prev_x_error_dt = 0
prev_y_error_dt = 0
def get_pid_values(x_input, y_input, x_set_point, y_set_point, kp, ki, kd, max_out):
    global prev_time, x_prev_error, y_prev_error, x_error_sum, y_error_sum, dt_sum, prev_x_error_dt, prev_y_error_dt
    
    temp_ki = 0.43
    alpha = 1
    
    x_error = x_input - x_set_point   # x location error calculation. it is different from y because reverse pid is necessary for the table
    x_error_dt = (x_error - x_prev_error)*alpha + prev_x_error_dt*(1-alpha) # x location error difference
    #print(x_error_dt)

    y_error = y_set_point - y_input
    y_error_dt = (y_error - y_prev_error)*alpha + prev_y_error_dt*(1-alpha)
    #print(y_error_dt)
    dt = 0.0
    curr_time = time.time()
 
    if prev_time is not None:
        dt = curr_time - prev_time
    prev_time = curr_time
    
    
    
    
    x_prev_error = x_error
    y_prev_error = y_error
    prev_x_error_dt = x_error_dt
    prev_y_error_dt = y_error_dt    
    if dt <= 0.0:
        return 0,0
    
    if abs(x_error) > 2:
        x_error_sum += ki*x_error*dt  # x_error integral calculation
        
    else:
        x_error_sum += temp_ki*x_error*dt
        #kp = 1
    if abs(y_error) > 2:
        y_error_sum += ki*y_error*dt  # x_error integral calculation
    else:
        y_error_sum += temp_ki*y_error*dt
        #kp = 1
    
    
    max_integral = 5
    
    #if abs(x_error)
    
    if (x_error_sum > max_integral):
        x_error_sum = max_integral
        
    elif (x_error_sum < -max_integral):
        x_error_sum = -max_integral

    if (y_error_sum > max_integral):
        y_error_sum = max_integral
        
    elif (y_error_sum < -max_integral):
        y_error_sum = -max_integral
    
    x_output = kp*x_error + x_error_sum + kd*x_error_dt/dt
    y_output = kp*y_error + y_error_sum + kd*y_error_dt/dt
    #print("pid_x: " + str(x_output) + " pid_y: " + str(y_output))
    
    """
    if abs(x_output) <= 2:
        x_output = 0
    if abs(y_output) <= 2:
        y_output = 0
    """
    # controlling pid max result
    if (x_output > max_out):
        x_output = max_out
        
    elif (x_output < -max_out):
        x_output = -max_out

    if (y_output > max_out):
        y_output = max_out
        
    elif (y_output < -max_out):
        y_output = -max_out
    #prev_time = curr_time

 

    return x_output, y_output




# this function is to initialize nrf24l01+
# nrf'i başlatmak için gerekli fonksiyon
def init_nrf():
    pipes = [0xE8, 0xE8, 0xF0, 0xF0, 0xE1]

    radio = NRF24(GPIO, spidev.SpiDev())
    radio.begin(0, 17) # spi0

    radio.setPayloadSize(32)
    radio.setChannel(0x77)
    radio.setDataRate(NRF24.BR_1MBPS)
    radio.setPALevel(NRF24.PA_MAX)

    #radio.setAutoAck(False)
    radio.enableDynamicPayloads()
    #radio.enableAckPayload()

    radio.openWritingPipe(pipes)
    #radio.openReadingPipe(1, pipes)
    radio.printDetails()
    #radio.startListening()
    radio.stopListening()
    
    return radio



# function required to send location telemetry
# konum telemetrisini göndermek için gerekli fonksiyon
def send_message(radio, ball):
    ball = ball + '*'
    radio.write(ball)



# this function returns center of the table
# to avoid fingers of the person or other thin things which can be detected as center prev_center algorithm is being used
# first when there is no disruptive things on the table center is found. First time center is found, 
# to change it distance of new circle's center from the center of circle whose center is first center points found should be +-16 pixels range 
# frame_masked -> masked frame according to mask_begin and mask_end
first_center = True
prev_center  = None
def find_table_center(frame, frame_masked):
    global first_center, prev_center
   
    circles = cv2.HoughCircles(frame_masked, cv2.HOUGH_GRADIENT, 1, 4000, param1 = 55, param2 = table_center_param2, minRadius = table_center_min_radius, maxRadius = table_center_max_radius)
    if circles is not None: 
        circles = np.uint16(np.around(circles))
        center_temp = circles[0,0,:]

        if first_center is True:
            first_center = False
            center = center_temp
        elif dist(center_temp[0], center_temp[1], prev_center[0], prev_center[1]) <= 10:
            center = center_temp
        else:
            center = prev_center
        
        prev_center = center  
        cv2.circle(frame, (center[0], center[1]), 1, (255, 0, 0), 3)
        cv2.circle(frame, (center[0], center[1]), center[2], (255, 255, 0), 3)
        return True, center     
    return False, prev_center





# this function is to find ball
# frame: rgb image
# frame_masked: gray scale image
# function finds the ball with hough transform
# hough dönüşümü ile top tespiti yapan fonksiyon
prev_circle = None
def find_ball(frame, frame_masked):
    
    global prev_circle 
    circles = cv2.HoughCircles(frame_masked, cv2.HOUGH_GRADIENT, 1, 2, param1 = 55, param2 = ball_center_param2, minRadius = ball_center_min_radius, maxRadius = ball_center_max_radius)
                                                                       # param1: canny edge detector threshold 
    
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None: chosen = i
            if prev_circle is not None:
                if dist(chosen[0], chosen[1], prev_circle[0], prev_circle[1]) <= dist(i[0], i[1], prev_circle[0], prev_circle[1]):
                    chosen = i
        prev_circle = chosen
        
        cv2.circle(frame, (chosen[0], chosen[1]), 1, (255, 0, 0), 3)
        cv2.circle(frame, (chosen[0], chosen[1]), chosen[2], (0, 0, 255), 3)
        #print( "X:" + str(chosen[0]) + "  Y:"  + str(chosen[1]) )
        
        #print(type(chosen[0]))
        
        return True, chosen
    #if prev_circle is None:
    return False, prev_circle
    #else:
        #return True, prev_circle





# this function is to take image and crop it
# görüntüyü alan ve gerekli kırpma işlemlerini yapan fonksiyon
FRAME_WIDTH = 300
TARGET_WIDTH = 50 # 500mm
def taking_frame():
    global frame, frame_gray, FRAME_WIDTH
    (grabbed, frame) = stream.read()
    frame = imutils.resize(frame[10:490, 100:580], width=FRAME_WIDTH)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)


# this function is to set servo degree from 0 to 180
# servo motors work with 50 Hz pwm signal, 0.5 ms is 0 degree and 2.5 ms is 180 degree
# servo motorlar 50 Hz'lik pwm sinyali ile çalışır
def set_servo_degree(pwm, servo, degree):
    pulse_w = 500 + (degree/180.0)*2000 # 2000 us -> 2 ms
    pwm.set_servo_pulsewidth(servo, pulse_w)



# this function is to draw location changes of the ball along all the process
# bütün süreç boyunca topun hareketini çizdiren fonksiyon
def draw_pid_result(x_points, y_points, time_arr, length, time_diff):
    #pass
    new_time = np.linspace(0, time_diff, length*10)
    f_x_smooth = interp1d(time_arr, x_points, kind = 'linear') # top konumundaki değişikliklere eğri uydurma
    f_y_smooth = interp1d(time_arr, y_points, kind = 'linear') # lineer interpolation
    #print(len(time_arr))
    #print(len(f_x_smooth))
    plt.figure(1)
    plt.plot(new_time, f_x_smooth(new_time))
    plt.title('X konum')
    plt.figure(2)
    plt.plot(new_time, f_y_smooth(new_time))
    plt.title('Y konum')
    plt.show()
    






# SERVO VARIABLES  #
x_servo = 12
x_pwm = pigpio.pi()
x_pwm.set_mode(x_servo, pigpio.OUTPUT)
x_pwm.set_PWM_frequency(x_servo, 50)

y_servo = 13
y_pwm = pigpio.pi()
y_pwm.set_mode(y_servo, pigpio.OUTPUT)
y_pwm.set_PWM_frequency(y_servo, 50)


SERVO_X_INITIAL_DEGREE = 72
SERVO_Y_INITIAL_DEGREE = 72



set_servo_degree(x_pwm, x_servo, SERVO_X_INITIAL_DEGREE)
set_servo_degree(y_pwm, y_servo, SERVO_Y_INITIAL_DEGREE)







MAX_OUT_OUTSIDE = 60.0
"""
Y_KP_OUTSIDE = 1.5#1.9
Y_KI_OUTSIDE = 1.0#0.9
Y_KD_OUTSIDE = 1.9#1.3
"""


Y_KP_OUTSIDE = 1.5#1.9
Y_KI_OUTSIDE = 0.9#0.9
Y_KD_OUTSIDE = 1.7#1.3



X_KP_OUTSIDE = -Y_KP_OUTSIDE
X_KI_OUTSIDE = -Y_KI_OUTSIDE
X_KD_OUTSIDE = -Y_KD_OUTSIDE

"""
SAMPLE_TIME = 0.015
"""


radio = init_nrf()

# taking image and finding table center
# görüntüyü alma ve tabla merkezinin bulma
taking_frame()
center_ret, center = find_table_center(frame, frame_gray)
X_SET_POINT = 25
Y_SET_POINT = 25

if center_ret is True:
    X_SET_POINT = center[1]*TARGET_WIDTH/FRAME_WIDTH
    Y_SET_POINT = center[0]*TARGET_WIDTH/FRAME_WIDTH
    print("Center x: " + str(X_SET_POINT) + "  Center y: " + str(Y_SET_POINT))


# variables for drawing ball location changes
x_points = [] 
y_points = []

begin_time = time.time()
stop_time = 0

prev_ball_x_nrf = 0
prev_ball_y_nrf = 0
while True:
    #a = datetime.datetime.now()
    taking_frame()
    
    ball_ret, ball = find_ball(frame, frame_gray)
    """
    ball_ret, ball2 = find_ball(frame, frame_gray)
    ball_ret, ball3 = find_ball(frame, frame_gray)
    """
    

    if ball_ret is True:
        """
        ball[0] = (ball[0]*0.1 + ball2[0]*0.2 + ball3[0]*0.7)
        ball[1] = (ball[1]*0.1 + ball2[1]*0.2 + ball3[1]*0.7)
        ball[2] = (ball[2]*0.1 + ball2[2]*0.2 + ball3[2]*0.7)
        """
        ball_x = ball[1]*TARGET_WIDTH/FRAME_WIDTH # mapping ball location
        ball_y = ball[0]*TARGET_WIDTH/FRAME_WIDTH
    
        ball_x_nrf = np.int32(ball_x)- np.int32(X_SET_POINT) # mapping ball location to between -25 and 25
        ball_y_nrf = np.int32(ball_y)- np.int32(Y_SET_POINT)
        
        x_points.append(ball_x_nrf) # adding data to drawing variables
        y_points.append(ball_y_nrf)
        
        # sending nrf data
        nrf_msg = "X:" + str(ball_x_nrf) + "  Y:"  + str(ball_y_nrf)
        send_message(radio, nrf_msg)
        
        # calculating pid output
        pid_out_x, pid_out_y = get_pid_values(ball_x, ball_y, X_SET_POINT, Y_SET_POINT, Y_KP_OUTSIDE, Y_KI_OUTSIDE, Y_KD_OUTSIDE, MAX_OUT_OUTSIDE)
        
        # adding initial servo degrees
        pid_out_x = pid_out_x + SERVO_X_INITIAL_DEGREE
        pid_out_y = pid_out_y + SERVO_Y_INITIAL_DEGREE
        
        print("pid_x: " + str(pid_out_x) + " pid_y: " + str(pid_out_y))
        
        # changing servo degrees
        
        set_servo_degree(x_pwm, x_servo, pid_out_x)
        set_servo_degree(y_pwm, y_servo, pid_out_y)
        prev_ball_x_nrf = ball_x_nrf
        prev_ball_y_nrf = ball_y_nrf
    else:
        # making table to flat when there is no ball on the table
        set_servo_degree(x_pwm, x_servo, SERVO_X_INITIAL_DEGREE)
        set_servo_degree(y_pwm, y_servo, SERVO_Y_INITIAL_DEGREE)
        prev_time = None
        x_prev_error = 0
        y_prev_error = 0
        x_error_sum = 0
        y_error_sum = 0
        

    #if ball_ret is True:
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_time = time.time()
            break
    #b = datetime.datetime.now()
    #print(b-a)


len_locations = len(x_points)
time_diff = stop_time-begin_time
time_arr = np.linspace(0, time_diff, len_locations)
length = len(time_arr)

#draw_pid_result(x_points, y_points, time_arr, length, time_diff)
cv2.destroyAllWindows()
stream.release()
GPIO.cleanup()



        