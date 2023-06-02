#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *
import pygame

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
Tips:
 press control+c to stop
----------------------------------------------------------
''')

start = True

def stop():
    global start
    start = False
    print('Closing...')
    set_velocity.publish(0, 0, 0)

def move_car(velocity_x, velocity_y):
    set_velocity.publish(velocity_x, velocity_y, 0)

if __name__ == '__main__':
    rospy.init_node('car_move_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Car Control")

    while start:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                start = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            move_car(500, 90)  # move forward
        elif keys[pygame.K_DOWN]:
            move_car(500, 270)  # move backward
        elif keys[pygame.K_LEFT]:
            move_car(500, 180)  # move left
        elif keys[pygame.K_RIGHT]:
            move_car(500, 0)  # move right
        else:
            move_car(0, 0)  # stop

        pygame.time.delay(10)

    move_car(0, 0)  # stop
    print('Closed')
    pygame.quit()
