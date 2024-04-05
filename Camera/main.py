import time
from pyb import UART, LED
import math
import sensor, image, time, struct
from math import *
import ulab

from ulab import numpy as np
import omv

#omv.disable_fb(True)
np_dot = np.dot
print( "version", ulab.__version__ )
led = LED(2) # green led
led.off()
serial = UART(3,500000)
'''Extended Kalman Filter for smoother ball following'''

class KalmanFilter:
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")
        self.n = F.shape[1]
        self.m = H.shape[1]
        self.F = F
        self.H = H
        #self.B = np.zeros(1, dtype=np.float) if B is None else B
        self.Q = np.eye(self.n, dtype=np.float) if Q is None else Q
        self.R = np.eye(self.n, dtype=np.float) if R is None else R
        self.P = np.zeros((self.n, self.n)) if P is None else P
        self.x = [[0], [0], [0], [0], [0], [0]]
    def predict(self):
        self.x = np_dot( self.F, self.x ) #+ np_dot( self.B, u )
        self.P = np_dot(np_dot(self.F, self.P), self.F.transpose().copy()) + self.Q
        return self.x
    def update(self, z):
        y = z - np_dot( self.H, self.x )
        S = self.R + np_dot( self.H, np_dot( self.P, self.H.transpose().copy() ) )
        K = np_dot( np_dot( self.P, self.H.transpose().copy() ), np.linalg.inv(S) )
        self.x = self.x + np_dot( K, y )
        I = np.eye( self.n )
        self.P = np_dot( np_dot( I - np_dot( K, self.H ), self.P ),
            (I - np_dot( K, self.H ) ).transpose().copy() ) + np_dot( np_dot( K, self.R ), K.transpose().copy() )
        return self.x

# set dT at each processing step
F = np.eye(6, dtype=np.float)
B = 0

H = np.array([[1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]],  dtype=np.float)


Q = np.array([[1e-2, 0, 0, 0, 0, 0],
            [0, 1e-2, 0, 0, 0, 0],
            [0, 0, 5.0 , 0, 0, 0],
            [0, 0, 0, 5.0 , 0, 0],
            [0, 0, 0, 0, 1e-2, 0],
            [0, 0, 0, 0, 0, 1e-2]], dtype=np.float)

R = np.array([[1e-1, 0, 0, 0],
            [0, 1e-1, 0, 0,],
            [0, 0, 1e-1, 0],
            [0, 0, 0, 1e-1]], dtype=np.float)

kf = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)



#setting = 'home'
#setting = 'lab'
setting = 'VincentHome'

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
if setting == 'lab':
    sensor.set_auto_gain(False, gain_db=22)
    #easier to detect blue goal but harder to yellow goal and ball if gain is higher
    #converse is true
elif setting == 'home' :
    sensor.set_auto_gain(False, gain_db =19)


print("1")
curr_gain = sensor.get_gain_db()

#sensor.set_auto_gain(False)
# === EXPOSURE ===
curr_exposure = sensor.get_exposure_us()
print(curr_exposure * 0.21)
print("2")
# if tuning exposure
#sensor.set_auto_exposure(False, exposure_us = 100000)

LAB_EXPOSURE = 1775
SC_EXPOSURE = 2000
#sensor.set_auto_exposure(False, exposure_us = LAB_EXPOSURE)

sensor.skip_frames(time = 1000)
# === WHITE BAL ===
#sensor.set_auto_whitebal(False, rgb_gain_db = (60, 60, 62)) #Must remain false for blob tracking
#sensor.set_auto_whitebal(False, rgb_gain_db = (4, 4, 4))
sensor.set_contrast(3)
#sensor.set_saturation(3)
#sensor.set_auto_exposure(False, exposure_us = 101243) #101243+
sensor.set_brightness(0)
print("3")
sensor.skip_frames(time=1000)
print("4")

clock = time.clock()

ID = 'robot1'
#ID = 'robot2'


if ID == 'robot2':
    centreAngleY = 128
    centreAngleX = 169
    centreY = 118
    centreX = 161

    ROI = (0, 0, 298, 240)

    if setting == 'lab':
        red_thresh = [(0, 100, 21, 127, 14, 88)]
        blue_thresh = [(0, 100, -128, 127, -128, -16)]
        yellow_thresh = [(0, 100, -11, 127, 24, 127)]

    elif (setting == 'home'):
        red_thresh = [(28, 72, 19, 127, 7, 36)]
        blue_thresh = [(0, 25, -119, -5, -122, -2)]
        yellow_thresh = [(0, 100, -11, 127, 14, 127)]
    elif setting == 'VincentHome':
        red_thresh = [(0, 100, 21, 127, 14, 88)]
        blue_thresh = [(0, 100, -128, 127, -128, -16)]
        yellow_thresh = [(0, 100, -11, 127, 24, 127)]




else:
    centreAngleY = 123
    centreAngleX = 162
    centreY = 123
    centreX = 162
    CAMERA_CENTER = np.array((centreX,centreY))


    if setting == 'lab':
        red_thresh = [(0, 100, 22, 127, -128, 127)]
        blue_thresh = [(0, 46, -128, 1, -128, -9)]
        yellow_thresh = [(0, 83, -128, 113, 18, 127)]

    elif (setting == 'home'):
        red_thresh = [(0, 100, 21, 127, 14, 88)]
        blue_thresh = [(0, 21, -128, 12, -122, -6)]
        yellow_thresh = [(0, 100, -11, 127, 14, 127)]
    elif setting == 'VincentHome':
        red_thresh = [(0, 100, 21, 127, 14, 88)]
        blue_thresh = [(0, 100, -128, 127, -128, -16)]
        yellow_thresh = [(0, 100, -11, 127, 24, 127)]


    ROI = (0, 0, 320, 240)




dT = 0
ballFound = False
notFoundCount = 0

"""
0 0
ZZ 32
20 53
30 64
40 75
50 83
60 90
70 95
80 102
90 105
100 107
110 108

"""
"""
0 0
10 24
12 26
13 30
15 34
17 38
19 41
22 46
25 51
28 55
30 58
33 62
35 64
40 69
45 74
50 77
55 80
60 83
65 85
70 87
75 89
80 91
85 93
90 94.5
100 97
110 99
120 104
160 106
180 109"""


# [ 1.19091531e-04 -1.03991140e-02  3.73862144e-01 -1.16801201e-01]
# 0.83628731 -24.12546642
def distanceMapper(pixel):
    # use polynomial regression to map pixel distance to centimetre distance
    polarity = pixel / abs(pixel) if pixel != 0 else 1
    # use absolute value since polynomial equation is different for negative numbers
    pixel = pixel

    #return (-1.16801201e-01+
           #(3.73862144e-01)  * pixel +
           #(-1.03991140e-02 ) * pixel**2 +
           #(1.19091531e-04) * pixel**3 ) * polarity
    return pixel


def distanceUnmapper(real):
    # use polynomial regression to map centimetre distance back to pixel distance
    polarity = real / abs(real) if real != 0 else 1
    # use absolute value since polynomial equation is different for negative numbers
    real = abs(real)
    return (-4.92497614 +
           (3.0761747478) * real +
           (-0.0238111740) * real**2 +
           (-0.0000109829) * real**3  +
           (0.0000008785) * real**4 +
           (-0.0000000027) * real**5 ) * polarity

class obj:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y

        #for angle only
        self.anglex = x
        self.angley = y
        self.w = w
        self.h = h
        self.area = w * h
        self.angle = 500
        self.dist = 500
        self.unmappedDist = 0
        self.confidence = 0

    def centralise(self):
        self.x -= centreX
        self.y = centreY - self.y

        self.anglex -= centreAngleX
        self.angley = centreAngleY - self.angley


    def process(self, mapDist = True):
        # currently ball is mapped before finding angle, goals are not
        # need to test to see if that makes a difference
        self.angle = degrees(atan2(self.angley,self.anglex))
        #if (self.x > 0 and self.y > 0):
            #self.angle = self.angle
        #elif (self.x > 0 and self.y < 0):
            #self.angle = self.angle + 90;
        #elif (self.x < 0 and self.y > 0):
            #self.angle = self.angle + 270;
        #elif (self.x < 0 and self.y < 0):
            #self.angle = 180 + self.angle
        #if self.angle < 0: self.angle += 360
        self.unmappedDist = sqrt(self.x ** 2 + self.y ** 2)
        if mapDist:
            self.dist = distanceMapper(sqrt(self.x ** 2 + self.y ** 2))
        else:
            self.dist = sqrt(self.x ** 2 + self.y ** 2)

        # ratio of distance to area should be fixed, allowing us to see if goal is obstructed.
        if (self.dist != 0):
            self.confidence = (self.area / self.dist) / 0.1


def track_field(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, roi = (0, 0, 320, 240), stride = 1, margin = 1):
    found_obj = None
    found_blob = None
    max_area = 0
    box = None
    # more blobs means more things blocking the goal
    num_blobs = 0
    blobs = img.find_blobs(thresh, merge=True, roi = roi, x_stride = 5, y_stride = 5, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        X = blob.cx()
        Y = blob.cy()
        #height = blob.h()
        box = blob.rect()
        for blob in blobs:
            num_blobs += 1
        if blob.area() > max_area:
            found_blob = blob


            max_area = blob.area()
            found_obj = obj(blob.cx(), blob.cy(), blob.w(), blob.h())

        if debug and found_obj:
            img.draw_rectangle(found_blob.rect(), color = color)
            img.draw_cross(found_blob.cx(), found_blob.cy(), color = color)
            #img.draw_line(centreX, found_blob.cy(), centreY, found_blob.cx(), color=color)
            found_obj.confidence = 1/num_blobs
        #img.draw_rectangle(box, color = color)

        #img.draw_cross(X, Y, color = color)


def track_obj(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, stride = 5, margin = 0, merge = False):
    found_obj = None
    found_blob = Nonex
    max_area = 0
    box = None
    # more blobs means more things blocking the goal
    num_blobs = 0
    blobs = img.find_blobs( [(0, 100, 22, 127, -128, 127)], merge=merge, roi = ROI, x_stride = stride, y_stride = stride, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        num_blobs += 1
        if blob.area() > max_area:
            found_blob = blob


            max_area = blob.area()
            found_obj = obj(blob.cx(), blob.cy(), blob.w(), blob.h())

    if debug and found_obj:
        img.draw_rectangle(found_blob.rect(), color = color)
        img.draw_cross(found_blob.cx(), found_blob.cy(), color = color)
        #img.draw_line(centreX, found_blob.cy(), centreY, found_blob.cx(), color=color)
        found_obj.confidence = 1/num_blobs

    return found_obj

def track_goal(thresh, pixel_thresh, area_thresh, color = (255, 255, 255), debug=False, stride = 5, margin = 0, merge = False):
    found_obj = None
    found_blob = None
    max_area = 0
    box = None
    # more blobs means more things blocking the goal
    num_blobs = 0
    blobs = img.find_blobs(thresh, merge=merge, roi = ROI, x_stride = stride, y_stride = stride, pixels_threshold=pixel_thresh, area_threshold=area_thresh, margin = margin)
    for blob in blobs:
        num_blobs += 1
        if blob.area() > max_area:
            found_blob = blob


            max_area = blob.area()
            found_obj = obj(blob.cx(), blob.cy(), blob.w(), blob.h())

    if debug and found_obj:
        img.draw_rectangle(found_blob.rect(), color = color)
        img.draw_cross(found_blob.cx(), found_blob.cy(), color = color)
        #img.draw_line(centreX, found_blob.cy(), centreY, found_blob.cx(), color=color)
        found_obj.confidence = 1/num_blobs

    return found_obj



def find_objects(debug=False):
    global ballFound
    global notFoundCount
    predBall = None
    img.draw_cross(centreX, centreY, color = (255, 255, 255))
    ball = track_goal(red_thresh, 3, 3, color = (0, 255, 0), stride = 1, debug =  debug, merge = False, margin = 0)
    blue = track_goal(blue_thresh, 5, 5, color = (0, 0, 255), stride = 2,  debug = debug, merge = True, margin = 0)
    yellow = track_goal(yellow_thresh, 10, 10, color = (255, 0, 0), stride = 5, debug =  debug, merge = False, margin = 0)

    if ballFound:
        kf.F[0][2] = dT
        kf.F[1][3] = dT

        state = kf.predict()
        predW = state[4][0]
        predH = state[5][0]
        predX = state[0][0]
        predY = state[1][0]
        predBall = obj(predX, predY, predW, predH)
        predBall.process(mapDist = False)

        if debug:
            debugPredX = centreX + distanceUnmapper(predX)
            debugPredY = centreY - distanceUnmapper(predY)


            #predRect = (int(debugPredX - predH / 2), int(debugPredY - predW / 2), int(predW), int(predH))
            #img.draw_rectangle(predRect, color = (0, 255, 255))
            #img.draw_cross(int(debugPredX), int(debugPredY), color = (0, 255, 255))


        # ball not detected but was recently seen

    if ball:
        notFoundCount = 0
        # map pixel dist to real dist so kalman filter can track ball more accurately
        #print(ball.x, ball.y)
        ball.centralise()
        ball.x = distanceMapper(ball.x)
        ball.y = distanceMapper(ball.y)

        z = np.array([[ball.x], [ball.y], [ball.w], [ball.h]], dtype=np.float)
        ball.process(mapDist = False)
        if not ballFound:
            # first detection!
            kf.P = np.eye(kf.F.shape[1])
            kf.x = np.array([z[0],z[1], [0], [0], z[2], z[3]], dtype=np.float)

            ballFound = True
        else:
            kf.update(z)
    else:
        # ball not detected
        notFoundCount += 1
        if notFoundCount >= 100:
            ballFound = False



    # create dummy objects if the object is not detected
    if yellow:
        yellow.centralise()
        yellow.process()
        #print(f"Yellow pixel dist: {yellow.x} ")
        #if debug:
            #print(f"Yellow Goal: Angle: {yellow.angle} Distance: {yellow.dist} Pixel Distance: {yellow.unmappedDist}")
    else:
        yellow = obj(0, 0, 0, 0)

    if blue:
        blue.centralise()
        blue.process()
        #print(f"blue pixel dist: {blue.y} ")

        #if debug:
            #print(f"Blue Goal: Angle: {blue.angle} Distance: {blue.dist} Pixel Distance: {blue.unmappedDist}")
    else:
        blue = obj(0, 0, 0, 0)
        #if debug:
            #print(f"Blue Goal not detected!")

    if ball:
        if debug:
            #print(f"Ball: Angle: {ball.angle} Distance: {ball.dist}")
            pass
    else:
        ball = obj(0, 0, 0, 0)

    if predBall:
        if debug:
            #print(f"Predicted Ball: Angle: {predBall.angle} Distance: {predBall.dist}")
            pass
    else:
        predBall = obj(0, 0, 0, 0)


    #return [ball.angle, ball.dist, predBall.angle, predBall.dist, blue.angle, blue.dist, yellow.angle, yellow.dist]
    return [blue.angle, blue.dist, yellow.angle, yellow.dist, ball.angle , ball.dist]

def send(data):
    sendData = [42]

    for num in data:
        # convert from 2dp float to int
        num = int(round(num, 2) * 100)


        sendData += list(num.to_bytes(2, 'little'))

    for num in sendData:
        try:
            uart.writechar(num)
        except:
            pass
def cobs_encode(input_bytes):
    read_index = 0
    write_index = 1
    code_index = 0
    code = 1

    output_bytes = bytearray(
        len(input_bytes) + 2
    )  # have enough space for the output data

    while read_index < len(input_bytes):
        if input_bytes[read_index] == 0:
            output_bytes[code_index] = code
            code = 1
            code_index = write_index
            write_index += 1
            read_index += 1
        else:
            output_bytes[write_index] = input_bytes[read_index]
            write_index += 1
            read_index += 1
            code += 1
            if code == 0xFF:
                output_bytes[code_index] = code
                code = 1
                code_index = write_index
                write_index += 1

    output_bytes[code_index] = code

    return output_bytes[:write_index]

while(True):
    debug = False
    debug = True

    clock.tick()
    img = sensor.snapshot()


    dT = 1/clock.fps()

    data = find_objects(debug=debug)
    time = clock.fps()
    #print(data)
        # Pack data
    buf = struct.pack(
        "<dddddd",
        data[0] ,  # d, double
        data[1] ,  # d, double
        data[2] ,  # d, double
        data[3],  # d, double
        data[4],
        data[5],
    )
    #buf = struct.pack(
        #"<dddddd",
        #data[0] ,  # d, double
        #data[1] ,  # d, double
        #data[2] ,  # d, double
        #data[3],  # d, double
        #data[4],
        #time,
    #)
    print(data[3])

    # Encode with COBS
    buf = cobs_encode(buf)
    buf += b"\x00"  # delimiter byte

    # Send packet
    serial.write(buf)
