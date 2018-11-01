#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
set_attitude_target.py: (Copter Only)
This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""
import dronekit_sitl
from dronekit import *
from pymavlink import mavutil

import time
import sys
import argparse
import math
import signal
from threading import Thread

################################
####### GLOBAL VARIABLES #######
################################
global vehicle              # object that holds the drone connection/info
vehicle = None

##################################
####### METHOD DEFINITIONS #######
##################################

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming the drone...")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    while(not vehicle.mode == 'GUIDED_NOGPS'):
        print "Waiting for guided no gps mode"
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        time.sleep(2)

    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(2)

    print "Ready to take off! (maybe not if safety switch is on...)"
    time.sleep(2)

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)


def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """

    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000, # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def show_info():
    # print " Type: %s" % vehicle.vehicle_type
    print " Armed: %s" % vehicle.armed
    print " Mode: %s" % vehicle.mode.name
    print " System status: %s" % vehicle.system_status.state
    print " Location: %s,%s" % (vehicle.location.global_relative_frame.lat,
                                vehicle.location.global_relative_frame.lon)
    print " Relative Alt: %s" % vehicle.location.global_relative_frame.alt
    print " Absolute Alt: %s" % vehicle.location.global_frame.alt


# Prints the location of the drone in a format to use with Google Earth
def print_location():
    lon = vehicle.location.global_frame.lon
    lat = vehicle.location.global_frame.lat
    alt = vehicle.location.global_frame.alt
    groundspeed = vehicle.groundspeed
    airspeed = vehicle.airspeed
    print "%s,%s,%s" % (lon,lat,alt)
    print "Airspeed: %s" % airspeed
    print "Groundspeed: %s" % groundspeed
    time.sleep(1)


# A simple pause to wait for the user to continue
def pause_for_input():
    raw_input("** PRESS ENTER TO CONTINUE **")

# Signal handler for when a keyboard interrupt occurs or program end
def end_program(*args):
    # Disarm the drone and wait before closing
    # tell to disarm False
    # while vehicle.armed:
    #     vehicle.armed = False
        # time.sleep(2)
    while (not vehicle.mode == 'RTL'):
        vehicle.mode = VehicleMode("RTL")

    # Close the connection to the drone
    vehicle.close()

    # if background_thread is not None:
    #     while background_thread.isAlive():
    #         print "Attempting to close threads"
    #         background_thread.join(1)
    #         time.sleep(1)

    print "Threads closed...program ending"
    print "Completed"
    quit()

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

#########################
####### MAIN CODE #######
#########################

# Set up interrupt handler for keyboard interrupt
signal.signal(signal.SIGINT, end_program)

connection_string = '/dev/ttyACM0'
baud_rate = 115200
print "Connecting to vehicle on: %s" % connection_string
# vehicle = connect(connection_string, wait_ready=True)
vehicle = connect(connection_string, baud=baud_rate)

 # Create message listener using decorator
@vehicle.on_message('*')
def listener(self, name, message):
     if name == 'COMMAND_ACK':
         if message.result != 0:
             print "COMMAND FAILED!"
         print 'message command: %s' % message.command
         print 'message results: %s' % message.result

show_info()

pause_for_input()

# Take off 2.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(2.5)
print "Ready to land"
pause_for_input()
# Hold the position for 3 seconds.
print("Hold position for 3 seconds")
set_attitude(duration = 3)

pause_for_input()
# # Uncomment the lines below for testing roll angle and yaw rate.
# # Make sure that there is enough space for testing this.
#
# # set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
# # set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
#
# # Move the drone forward and backward.
# # Note that it will be in front of original position due to inertia.
# print("Move forward")
# set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)
#
# print("Move backward")
# set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)
#

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
while True:
    alt = vehicle.location.global_relative_frame.alt
    print "Altitude: %s" % alt
    if alt <= 0.5:
        print "Landed"
        break
    time.sleep(0.5)

time.sleep(1)

show_info()
pause_for_input()

#Script is completed
end_program()
