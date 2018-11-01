# Import DroneKit-Python and Simulator
import dronekit_sitl
from dronekit import *
from pymavlink import mavutil

import time
import sys
import argparse
import math
import signal
from threading import Thread

######################################################################
##                                                                  ##
##                         GLOBAL VARIABLES                         ##
##                                                                  ##
######################################################################
global vehicle
vehicle = None

######################################################################
##                                                                  ##
##                        METHOD DEFINITIONS                        ##
##                                                                  ##
######################################################################

# Arm the vehicle and takeoff to target altitude in meters
def arm_and_takeoff(aTargetAltitude):
    # Don't arm until autopilot is ready
    print "Basic pre-arm checks..."
    while not vehicle.is_armable:
        print "Waiting for vehicle to initialize..."
        time.sleep(2)

    # Set the drone to guided mode and confirm
    print "Setting mode to GUIDED"
    while not vehicle.mode == 'GUIDED':
        vehicle.mode = VehicleMode("GUIDED")
        print "Waiting for GUIDED mode"
        time.sleep(2)

    # Arm the drone and confirm
    print "Arming the drone..."
    while not vehicle.armed:
        vehicle.armed = True
        print "Waiting for arming..."
        time.sleep(2)

    print "Ready to take off! (maybe not if safety switch is on...)"
    time.sleep(2)

    # Take off to target altitude
    print "Taking off"
    timer = 0
    while timer <= 2:
        vehicle.simple_takeoff(aTargetAltitude)
        timer += 1
        time.sleep(3)

    # Wait until the drone reaches at least 95% of the target
    # altitude before running other commands
    while True:
        relative_alt = vehicle.location.global_relative_frame.alt
        absolute_alt = vehicle.location.global_frame.alt
        print "Relative Altitude: %s" % relative_alt
        # print "Absolute Altitude: %s" % absolute_alt

        if relative_alt >= aTargetAltitude*0.95:
            print "Reached target altitude"
            break

        time.sleep(1)

# Display some basic state information
def show_info():
    # print " Type: %s" % vehicle.vehicle_type
    print " Armed: %s" % vehicle.armed
    print " Mode: %s" % vehicle.mode.name
    print " System status: %s" % vehicle.system_status.state
    print " GPS: %s" % vehicle.gps_0
    print " Location: %s,%s" % (vehicle.location.global_relative_frame.lat,
                                vehicle.location.global_relative_frame.lon)
    print " Relative Alt: %s" % vehicle.location.global_relative_frame.alt
    print " Absolute Alt: %s" % vehicle.location.global_frame.alt

# Prints the location of the drone in a format to use with Google Earth
def print_location():
    while True:
        lon = vehicle.location.global_frame.lon
        lat = vehicle.location.global_frame.lat
        alt = vehicle.location.global_frame.alt
        groundspeed = vehicle.groundspeed
        airspeed = vehicle.airspeed
        print "%s,%s,%s" % (lon,lat,alt)
        print "Airspeed: %s" % airspeed
        print "Groundspeed: %s" % groundspeed
        time.sleep(1)

# Check to see if drone has arrived at the given location
def wait_for_arrival(lat, lon):
    count = 0
    while count < 60:
        latitude = vehicle.location.global_relative_frame.lat
        longitude = vehicle.location.global_relative_frame.lon
        altitude = vehicle.location.global_relative_frame.alt
        # print "%Velocity: %s" % vehicle.velocity

        if (latitude >= lat-0.000005 and latitude <= lat+0.000005
        and longitude >= lon-0.000005 and longitude <= lon+0.000005):
            print "Arrived"
            break

        count += 1
        time.sleep(1)


# A simple pause to wait for the user to continue
def pause_for_input():
    raw_input("** PRESS ENTER TO CONTINUE **")

# Signal handler for when a keyboard interrupt occurs or program end
def end_program(*args):
    # Disarm the drone and wait before closing
    # while vehicle.armed:
    #     vehicle.armed = False
    #     time.sleep(2)

    # Set the mode to RTL (return to land) and confirm
    while not vehicle.mode == 'RTL':
        vehicle.mode = VehicleMode("RTL")
        time.sleep(2)

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

######################################################################
##                                                                  ##
##                             MAIN CODE                            ##
##                                                                  ##
######################################################################

# Set up interrupt handler
signal.signal(signal.SIGINT, end_program)

# Start simulation and connect to the drone
# HOME LOCATION (Test Site): 35.970264,-79.091461
# SITL command: dronekit-sitl copter-3.3 --home=35.970264,-79.091461,170,353
print "Start simulator (SITL)"
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# connection_string = 'tcp:127.0.0.1:5760'
# connection_string = 'com3'
# baud_rate = 57600
print "Connecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True)
# vehicle = connect(connection_string, baud=baud_rate)

#Create a message listener for COMMAND_ACK messages.
moveOnFlag = False
errorCount = 0
MAX_ERROR_COUNT = 20
@vehicle.on_message('COMMAND_ACK')
def listener1(self, name, message):
    print 'commandId: %s' % message.command
    def resultTable(x):
        return {
            0 : 'Command ACCEPTED and EXECUTED'
            1 : 'Command TEMPORARY REJECTED/DENIED'
            2 : 'Command PERMANENTLY DENIED'
            3 : 'Command UNKNOWN/UNSUPPORTED'
            4 : 'Command executed, but failed'
            5 : 'WIP: Command being executed'
        }.get(x, 'Unknown Result')
    print 'result: %s' % resultTable(message.result)
    if message.result == 0 :
        moveOnFlag = True
        errorCount = 0
    else :
        errorCount++
@vehicle.on_message('ACTION_ACK')
def listener2(self, name, message):
    print 'message: %s' % message
show_info()

pause_for_input()

# Takeoff to the given altitude in meters
alt = 10
while not moveOnFlag:
    if errorCount > MAX_ERROR_COUNT :
        end_program()
    arm_and_takeoff(alt)
    time.sleep(0.5)
moveOnFlag = False
home_loc = vehicle.location
home_lat = home_loc.global_frame.lat
home_lon = home_loc.global_frame.lon

pause_for_input()

speed = 5         # airspeed in m/s

# Goto first target location
print "Going to first target location"
# background_thread = Thread(target=print_location)
# background_thread.setDaemon(True)
# background_thread.start()
lat = 35.971170
lon = -79.091960
location1 = LocationGlobalRelative(lat, lon, alt+5)
while not moveOnFlag:
    if errorCount > MAX_ERROR_COUNT :
        end_program()
    vehicle.simple_goto(location1, groundspeed=speed)
    time.sleep(0.5)
moveOnFlag = False
wait_for_arrival(lat, lon)

# while background_thread.isAlive():
#     background_thread.paused = True
#     time.sleep(1)

pause_for_input()

# Goto second target location
print "Going to second target location"
lat = 35.969751
lon = -79.092166
location2 = LocationGlobalRelative(lat, lon, alt)
while not moveOnFlag:
    if errorCount > MAX_ERROR_COUNT :
        end_program()
    vehicle.simple_goto(location2, airspeed=speed)
    time.sleep(0.5)
moveOnFlag = False
wait_for_arrival(lat, lon)

pause_for_input()

# Return to the landing zone
print "Returning to the landing zone"
while not vehicle.mode == VehicleMode("RTL")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(0.5)
# wait_for_arrival(home_lat, home_lon)

# print "Ready to land"
# pause_for_input()

while True:
    alt = vehicle.location.global_relative_frame.alt
    print "Altitude: %s" % alt
    if alt <= 0.05:
        print "Landed"
        break
    time.sleep(0.5)

show_info()
pause_for_input()

# Script is completed
end_program()