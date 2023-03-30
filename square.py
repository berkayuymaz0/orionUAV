#dronekit library is used to make square movement of drone in python
from dronekit import connect, VehicleMode, LocationGlobalRelative,Command,mavutil
import time
import math
import argparse


#connection string is used to connect to the drone
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
#altitude is used to set the altitude of the drone
alt = 10
#function to make the drone move in square
def move_square():
    #radius is used to set the radius of the square
    radius = 0.0001
    #lat is used to set the latitude of the drone
    lat = vehicle.location.global_relative_frame.lat
    #lon is used to set the longitude of the drone
    lon = vehicle.location.global_relative_frame.lon
    #while loop is used to check if the altitude of the drone is less than the altitude set
    while True:
        if vehicle.location.global_relative_frame.alt >= alt * 0.95:
            break
        time.sleep(1)
    #points is used to set the number of points in the square
    points = 4
    #loops is used to set the number of loops
    loops = 2 * points
    #coords is used to set the coordinates of the drone
    coords = []
    #for loop is used to set the coordinates of the drone
    for i in range(0, loops):
        degrees = (i/points)*360
        radians = (math.pi/180)*degrees
        x = lat + radius * math.cos(radians)
        y = (lon + radius * math.sin(radians))
        coords.append((x,y))
    #cmds is used to set the commands of the drone
    cmds = vehicle.commands
    #clear is used to clear the commands of the drone
    cmds.clear()
    #for loop is used to set the commands of the drone
    for lat,lon in coords:
        point = LocationGlobalRelative(lat,lon,alt)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))
    #upload is used to upload the commands to the drone
    cmds.upload()
    #mode is used to set the mode of the drone
    vehicle.mode = VehicleMode("AUTO")
    #sleep is used to set the time of the drone
    time.sleep(1)
    #while loop is used to check if the next waypoint is reached
    while True:
        nextwaypoint=vehicle.commands.next
        time.sleep(1)
#function to make the drone take off
def takeoff(alt):
    #while loop is used to check if the drone is armable
    while vehicle.is_armable is not True:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    #mode is used to set the mode of the drone
    vehicle.mode = VehicleMode("GUIDED")
    #armed is used to arm the drone
    vehicle.armed = True
    #while loop is used to check if the drone is armed
    while vehicle.armed is not True:
        print("Arming motors")
        time.sleep(0.5)
    #simple_takeoff is used to make the drone take off
    vehicle.simple_takeoff(alt)
    #while loop is used to check if the drone has reached the altitude set
    while vehicle.location.global_relative_frame.alt < alt * 0.95:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    print("Reached target altitude")
#function to make the drone land
def land():
    #mode is used to set the mode of the drone
    vehicle.mode = VehicleMode("LAND")
    #while loop is used to check if the drone has landed
    while vehicle.armed is True:
        print("Landing")
        time.sleep(1)
#function to make the drone disarm
def disarm():
    #armed is used to disarm the drone
    vehicle.armed = False
    #while loop is used to check if the drone is disarmed
    while vehicle.armed is True:
        print("Disarming")
        time.sleep(1)
#function to make the drone RTL
def RTL():
    #mode is used to set the mode of the drone
    vehicle.mode = VehicleMode("RTL")
    #while loop is used to check if the drone has returned to launch
    while vehicle.armed is True:
        print("Returning to launch")
        time.sleep(1)
#function to make the drone arm
def arm():
    #armed is used to arm the drone
    vehicle.armed = True
    #while loop is used to check if the drone is armed
    while vehicle.armed is not True:
        print("Arming motors")
        time.sleep(0.5)

takeoff(alt)
move_square()
land()
disarm()
vehicle.close()
