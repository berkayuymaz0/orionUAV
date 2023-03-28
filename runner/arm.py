import math
import random
import sys
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative, LocationGlobal
from math import cos, asin, sqrt, sin, pi, radians

import cv2


##### DRONE #####
class Drone:
    def __init__(self, orion, sim=False):
        self.orion = orion
        self.sim = sim
        self.command = orion.commands

        self.mask_dropped = False

        self.reset()

    def reset(self):
        self.mask_dropped = False

    def takeoff(self, attitude):
        while self.orion.is_armable is not True:
            print("Waiting for vehicle to initialise...")
            time.sleep(1)
        self.orion.mode = VehicleMode("GUIDED")

        self.orion.armed = True

        while self.orion.armed is not True:
            print("Arming motors")
            time.sleep(0.5)

        print("Orion Armed...")
        self.orion.simple_takeoff(attitude)
        print("Taking off!.")
        while self.orion.location.global_relative_frame.alt < attitude * 0.95:
            print(" Altitude: ", orion.location.global_relative_frame.alt)
            time.sleep(1)

    def change_mode(self, mode_n):
        while self.orion.mode != VehicleMode(mode_n):
            print(f"changing to {mode_n} .")
            self.orion.mode = VehicleMode(mode_n)
            time.sleep(1)

    def up(self, alt):
        while self.orion.location.global_relative_frame.alt > alt * 0.9:
            tmpLoc = LocationGlobalRelative(
                self.orion.location.global_relative_frame.lat,
                self.orion.location.global_relative_frame.lon,
                alt)

            self.orion.simple_goto(tmpLoc)
            time.sleep(1)
            print("Going higher..")
            if self.orion.location.global_relative_frame.alt < alt + 0.3:
                return True

    def failsafe(self):
        if self.orion.mode == VehicleMode("RTL"):
            print("Coming Back to Home")
            sys.exit(1)

    def get_distance(self, lat1, lon1, lat2, lon2):
    #distance between two location
        p = 0.017453292519943295     #Pi/180
        a = 0.5 - cos((lat2 - lat1) * p)/2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
        return 12742 * asin(sqrt(a)) * 1000                


    def get_location_metres(original_location, dNorth, dEast):
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*cos(pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/pi)
        newlon = original_location.lon + (dLon * 180/pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
        
        return targetlocation

    def send_ned_position(self,d_north, d_east):

        msg = self.orion.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            d_north, d_east, 0, # x, y, z positions 
            0,0,0, # x, y, z velocity in m/s (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        self.orion.send_mavlink(msg)
        time.sleep(2)
        print("success")

##### RUNNER ######

orion = connect('tcp:127.0.0.1:5762', wait_ready=True)

drone = Drone(orion, sim=False)


#takeoff
time.sleep(5)
drone.takeoff(10)
time.sleep(5)

while True:

    fire = None

    if fire is not None:


