import math
import random
import sys
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil


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
        while self.orion.location.global_relative_frame.alt > alt * 0.8:
            tmpLoc = LocationGlobalRelative(
                self.orion.location.global_relative_frame.lat,
                self.orion.location.global_relative_frame.lon,
                alt)

            self.orion.simple_goto(tmpLoc)
            time.sleep(1)
            print("Going lower..")
            if self.orion.location.global_relative_frame.alt < alt + 0.3:
                return True

    def failsafe(self):
        if self.orion.mode == VehicleMode("RTL"):
            print("Coming Back to Home")
            sys.exit(1)


##### RUNNER ######

orion = connect('tcp:127.0.0.1:5762', wait_ready=True)

drone = Drone(orion, sim=False)

drone.takeoff(10)
time.sleep(5)
drone.up(10)

