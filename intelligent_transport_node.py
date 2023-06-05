#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import time
import math
import smbus
import rospy
import numpy as np
from threading import RLock, Timer, Thread

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from chassis_control.msg import *
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from intelligent_transport.srv import SetTarget
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform




"""

"""
class TTS:
      
    address = 0x40
    bus = None

    def __init__(self, bus=1):
        self.bus = smbus.SMBus(bus)
    
    def WireReadTTSDataByte(self):
        try:
            val = self.bus.read_byte(self.address)
        except:
            return False
        return True
    
    def TTSModuleSpeak(self, sign, words):
        head = [0xFD, 0x00, 0x00, 0x01, 0x00]             # Text playback command
        wordslist = words.encode("gb2312")            # Set the text encoding format to GB2312
        signdata = sign.encode("gb2312")
        length = len(signdata) + len(wordslist) + 2
        head[1] = length >> 8
        head[2] = length
        head.extend(list(signdata))
        head.extend(list(wordslist))
        try:
            self.bus.write_i2c_block_data(self.address, 0, head) # Send data to the slave
        except:
            print('Sensor not connected!')
        time.sleep(0.05)



# 自主搬运

lock = RLock()
ik = ik_transform.ArmIK()

set_visual = 'line'
detect_step = 'color'  # Step: line following or color detection
line_color = 'yellow'  # Line color
stable = False        # Color block gripping flag
place_en = False      # Color block placement flag
position_en = False   # Color block positioning flag
__isRunning = False   # Gameplay control switch
block_clamp = False   # Color block transport flag
chassis_move = False  # Chassis movement flag
x_dis = 500
y_dis = 0.15
line_width = 0
line_center_x = 0
line_center_y = 0
color_centreX = 320
color_centreY = 410
color_center_x = 0
color_center_y = 0
detect_color = 'None'  
target_color = 'None'

img_h, img_w = 480, 640

line_x_pid = PID.PID(P=0.002, I=0.001, D=0)  # PID initialization
color_x_pid = PID.PID(P=0.06, I=0, D=0) 
color_y_pid = PID.PID(P=0.00003, I=0, D=0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'gray' : (50,50,50),
    'yellow': (0, 255, 255),
    'white': (255, 255, 255),
    'purple':(160, 32 , 240),
}

# Initial position
def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500)))
    if delay:
        rospy.sleep(2)

# Turn off RGB lights
def off_rgb():
    global rgb_pub

    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)


# Reset variables
def reset():
    global x_dis, y_dis
    global position_en, stable
    global set_visual, detect_step, place_en
    global block_clamp, chassis_move, target_color
    global line_width, line_center_x, line_center_y
    global detect_color, color_center_x, color_center_y

    
    with lock:
        line_x_pid.clear()
        color_x_pid.clear()
        color_y_pid.clear()
        off_rgb()
        set_visual = 'line'
        detect_step = 'color'
        stable = False
        place_en = False
        position_en = False
        block_clamp = False
        chassis_move = False
        x_dis = 500
        y_dis = 0.15
        line_width = 0
        line_center_x = 0
        line_center_y = 0
        color_center_x = 0
        color_center_y = 0
        detect_color = 'None'
        target_color = 'None'
        
        

# Initialization call
def init():
    rospy.loginfo("intelligent transport Init")
    initMove()
    reset()


n = 0
last_x = 0
last_y = 0
# Image processing result callback function
def run(msg):
    global lock, n, last_x, last_y, position_en
    global line_width, line_center_x, line_center_y
    global detect_color, color_center_x, color_center_y

    data = int(msg.data)
    center_x = msg.center_x
    center_y = msg.center_y
    color_list = {0: 'None', 1: 'red', 2: 'green', 3: 'blue', 4: 'gray', 5: 'purple'}

    with lock:
        # Update line or color block position parameters
        if detect_step == 'line':
            line_center_x = center_x
            line_center_y = center_y
            line_width = data

            color_center_x = 0
            color_center_y = 0
            detect_color = color_list[0]

        elif detect_step == 'color':
            color_center_x = center_x
            color_center_y = center_y
            data = 0 if data < 0 else data
            data = 0 if data > 5 else data
            detect_color = color_list[data]

            if not position_en:  # Check if the color block is stable
                dx = abs(color_center_x - last_x)
                dy = abs(color_center_y - last_y)
                last_x = color_center_x
                last_y = color_center_y
                if dx < 3 and dy < 3:
                    n += 1
                    if n == 10:
                        n = 0
                        position_en = True  # Stable position
                else:
                    n = 0

            line_center_x = 0
            line_center_y = 0
            line_width = 0


# Robot movement function
def move():
    global x_dis, y_dis
    global position_en, stable
    global set_visual, detect_step, place_en
    global block_clamp, chassis_move, target_color
    global line_width, line_center_x, line_center_y
    global detect_color, color_center_x, color_center_y
    global garbage_species
    global offset


    
    num = 0
    move_time = time.time()
    place_delay = time.time()
    transversae_time = time.time()

    set_visual = 'color'
    detect_step = 'line'

    while __isRunning:
        if detect_step=='line':  # Line following stage
            if set_visual=='color':
                set_visual='line'
                place_en=False
                visual_running('line', line_color)  # Switch image processing type
                # Switch mechanical arm posture
                bus_servo_control.set_servos(joints_pub, 1500,
                                             ((1, 500), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
                rospy.sleep(1.5)

            elif line_width > 0:  # Line detected
                # PID algorithm for line following
                if abs(line_center_x-img_w / 2) < 30:
                    line_center_x=img_w / 2
                line_x_pid.SetPoint=img_w / 2  # Setpoint
                line_x_pid.update(line_center_x)  # Current value
                dx=round(line_x_pid.output, 2)  # Output
                dx=0.8 if dx > 0.8 else dx
                dx=-0.8 if dx < -0.8 else dx

                set_velocity.publish(100, 90, dx)  # Control chassis
                chassis_move=True

                if not block_clamp:  # place_en:
                    if line_width > 100:  # Detect horizontal line
                        rospy.sleep(1)
                        set_velocity.publish(0, 0, 0)
                        rospy.sleep(0.1)
                        detect_step='color'




        elif detect_step=='color':  # Color detection stage
            if set_visual=='line':
                x_dis=500
                y_dis=0.15
                stable=False
                set_visual='color'
                visual_running('colors', 'rgb')  # Switch image processing type
                # Switch mechanical arm posture
                target=ik.setPitchRanges((0, 0.15, 0.03), -180, -180, 0)
                if target:
                    servo_data=target[1]
                    bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                                                                    (4, servo_data['servo4']),
                                                                    (5, servo_data['servo5']), (
                                                                    6, servo_data['servo6'])))  # Mechanical arm posture
                    rospy.sleep(1.5)

            elif detect_color!='None' and not block_clamp:  # Color block is stable, perform tracking and gripping
                if position_en:
                    diff_x=abs(color_center_x-color_centreX)
                    diff_y=abs(color_center_y-color_centreY)
                    # X-axis PID tracking
                    if diff_x < 10:
                        color_x_pid.SetPoint=color_center_x  # Setpoint
                    else:
                        color_x_pid.SetPoint=color_centreX
                    color_x_pid.update(color_center_x)  # Current value
                    dx=color_x_pid.output  # Output
                    x_dis+=int(dx)
                    x_dis=200 if x_dis < 200 else x_dis
                    x_dis=800 if x_dis > 800 else x_dis
                    # Y-axis PID tracking
                    if diff_y < 10:
                        color_y_pid.SetPoint=color_center_y  # Setpoint
                    else:
                        color_y_pid.SetPoint=color_centreY
                    color_y_pid.update(color_center_y)  # Current value
                    dy=color_y_pid.output  # Output
                    y_dis+=dy
                    y_dis=0.12 if y_dis < 0.12 else y_dis
                    y_dis=0.28 if y_dis > 0.28 else y_dis

                    # 机械臂追踪移动到色块上方
                    target = ik.setPitchRanges((0, round(y_dis, 4), 0.03), -180, -180, 0)
                    if target:
                        servo_data = target[1]
                        bus_servo_control.set_servos(joints_pub, 20,((3, servo_data['servo3']),         
                             (4, servo_data['servo4']),(5, servo_data['servo5']), (6, x_dis)))
                    
                    if dx < 2 and dy < 0.003 and not stable: # 等待机械臂稳定停在色块上方
                        num += 1
                        if num == 10:
                            stable = True  # 设置可以夹取
                            num = 0
                    else:
                        num = 0

                    if stable_flag:  # Control the robotic arm for gripping
                        offset=0.02
                        offset_y=Misc.map(target[2], -180, -150, -0.03, 0.03)
                        set_rgb(detect_color)  # Set RGB light color
                        target_color=detect_color  # Store the target color
                        buzzer_pub.publish(0.1)  # Activate the buzzer

                        bus_servo_control.set_servos(joints_pub, 500, ((1, 120),))  # Open the gripper
                        rospy.sleep(0.5)
                        target=ik.setPitchRanges((0, (round(y_dis+offset_y, 5)+offset), -0.07), -180, -180,
                                                 0)  # Extend the arm downward
                        if target:
                            servo_data=target[1]
                            bus_servo_control.set_servos(joints_pub, 1000, ((3, servo_data['servo3']),
                                                                            (4, servo_data['servo4']),
                                                                            (5, servo_data['servo5']), (6, x_dis)))
                        rospy.sleep(1.5)
                        bus_servo_control.set_servos(joints_pub, 500, ((1, 500),))  # Close the gripper
                        rospy.sleep(0.8)

                        bus_servo_control.set_servos(joints_pub, 1500, (
                        (1, 500), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))  # Lift the arm up
                        rospy.sleep(1.5)

                        if target_color=='red':
                            print("Hazardous waste")
                            tts.TTSModuleSpeak("[h0][v10][m52]", "Hazardous waste")
                            bus_servo_control.set_servos(joints_pub, 1200, ((6, 774),))  # Rotate the arm first
                            rospy.sleep(1)
                            bus_servo_control.set_servos(joints_pub, 1500, (
                                (3, 200), (4, 914), (5, 752)))  # Then lower it down
                            rospy.sleep(1.8)
                            bus_servo_control.set_servos(joints_pub, 500, ((1, 150),))  # Open the gripper
                            rospy.sleep(0.8)

                        elif target_color=='green':
                            print("Kitchen Waste")
                            tts.TTSModuleSpeak("[h0][v10][m52]", "Kitchen Waste")
                            bus_servo_control.set_servos(joints_pub, 1200, ((6, 223),))  # Rotate the robotic arm first
                            rospy.sleep(1)
                            bus_servo_control.set_servos(joints_pub, 1500,
                                                         ((3, 200), (4, 914), (5, 752)))  # Lower the robotic arm
                            rospy.sleep(1.8)
                            bus_servo_control.set_servos(joints_pub, 500, ((1, 150),))  # Open the gripper
                            rospy.sleep(0.8)

                        elif target_color=='blue':
                            print("Recyclable Waste")
                            tts.TTSModuleSpeak("[h0][v10][m52]", "Recyclable Waste")
                            bus_servo_control.set_servos(joints_pub, 1200, ((6, 1000),))  # Rotate the robotic arm first
                            rospy.sleep(1)
                            bus_servo_control.set_servos(joints_pub, 1500,
                                                         ((3, 200), (4, 914), (5, 752)))  # Lower the robotic arm
                            rospy.sleep(1.8)
                            bus_servo_control.set_servos(joints_pub, 500, ((1, 150),))  # Open the gripper
                            rospy.sleep(0.8)

                        elif target_color=='gray':
                            print("Other Waste")
                            tts.TTSModuleSpeak("[h0][v10][m52]", "Other Waste")
                            bus_servo_control.set_servos(joints_pub, 1200, ((6, 0),))  # Rotate the robotic arm first
                            rospy.sleep(1)
                            bus_servo_control.set_servos(joints_pub, 1500,
                                                         ((3, 200), (4, 914), (5, 752)))  # Lower the robotic arm
                            rospy.sleep(1.8)
                            bus_servo_control.set_servos(joints_pub, 500, ((1, 150),))  # Open the gripper
                            rospy.sleep(0.8)

                        #elif detect_color == 'purple':
                         #   print("purple")

                            # Reset the robotic arm
                        bus_servo_control.set_servos(joints_pub, 1500,
                                                         ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625)))
                        rospy.sleep(1.5)
                        bus_servo_control.set_servos(joints_pub, 1500, ((6, 500),))
                        rospy.sleep(1.5)

                # Reset variables
                        reset()

                else:
                    rospy.sleep(0.01)

        else:
            rospy.sleep(0.01)

# Set RGB light color
def set_rgb(color):
    global lock
    with lock:
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[color][2]
        led.rgb.g = range_rgb[color][1]
        led.rgb.b = range_rgb[color][0]
        rgb_pub.publish(led)
        rospy.sleep(0.05)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.1)


result_sub=None
heartbeat_timer=None


# Callback function for 'enter' service
def enter_func(msg):
    global lock
    global result_sub

    rospy.loginfo("Enter intelligent transport")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub=rospy.Subscriber('/visual_processing/result', Result, run)

    return [True, 'enter']


# Callback function for 'exit' service
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer

    rospy.loginfo("Exit intelligent transport")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning=False
        reset()
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub=None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer=None

        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']


# Start running function
def start_running():
    global lock
    global __isRunning

    rospy.loginfo("Start running intelligent transport")
    with lock:
        init()
        __isRunning=True
        rospy.sleep(0.1)
        # Run in a separate thread
        th=Thread(target=move)
        th.setDaemon(True)
        th.start()


# Stop running function
def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("Stop running intelligent transport")
    with lock:
        reset()
        __isRunning=False
        initMove(delay=False)
        set_velocity.publish(0, 0, 0)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()


# set_running service callback function
def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']


# heartbeat service callback function
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer=Timer(5, rospy.ServiceProxy('/intelligent_transport/exit', Trigger))
        heartbeat_timer.start()
    rsp=SetBoolResponse()
    rsp.success=msg.data

    return rsp


if __name__ == '__main__':
    tts = TTS()
    # Initialize node
    rospy.init_node('intelligent_transport', log_level=rospy.DEBUG)
    # Visual processing
    visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
    # Servo publisher
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # App communication services
    enter_srv = rospy.Service('/intelligent_transport/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/intelligent_transport/exit', Trigger, exit_func)
    running_srv = rospy.Service('/intelligent_transport/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/intelligent_transport/heartbeat', SetBool, heartbeat_srv_cb)
    # Chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # Buzzer
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # RGB LED
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    rospy.sleep(0.5)  # Delay is required for the publishers to take effect

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

