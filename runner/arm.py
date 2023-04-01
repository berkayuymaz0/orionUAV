import sys
import time
import math
from math import cos, asin, sqrt, pi
import torch

import cv2
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil



##### DRONE #####
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * cos(pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / pi)
    newlon = original_location.lon + (dLon * 180 / pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def get_distance(lat1, lon1, lat2, lon2):
    # distance between two location
    p = 0.017453292519943295  # Pi/180
    a = 0.5 - cos((lat2 - lat1) * p) / 2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
    return 12742 * asin(sqrt(a)) * 1000


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



####AI####
def load_model(model_name):
    if model_name:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_name, force_reload=True)
    else:
        model = torch.hub.load('/yolov5', 'yolov5s', pretrained=True)
    return model


class live_ai:

    def __init__(self, capture_index, model_name):

        self.capture_index = capture_index
        self.model = load_model(model_name)
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

    def get_video_capture(self):

        return cv2.VideoCapture(self.capture_index)

    def score_frame(self, frame):

        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord

    def class_to_label(self, x):
        return self.classes[int(x)]

    def plot_boxes(self, results, frame):

        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.3:
                x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(
                    row[3] * y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
                if detector.class_to_label(labels[i]) == 'person':
                    fire_found = True
                    print("fire detected")
                if detector.class_to_label(labels[i]) == 'house':
                    house_found = True
                    print("house detected")
                else:
                    fire_found = False
                    house_found = False
                    print("no fire ,no house detected")

        return frame

    def __call__(self):
        cap = self.get_video_capture()
        assert cap.isOpened()

        while True:

            ret, frame = cap.read()
            assert ret

            frame = cv2.resize(frame, (416, 416))

            start_time = time.time()
            results = self.score_frame(frame)
            frame = self.plot_boxes(results, frame)

            end_time = time.time()
            fps = 1 / np.round(end_time - start_time, 2)

            cv2.putText(frame, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)

            cv2.imshow('YOLOv5 Detection', frame)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def move_sircle(alt): 
    radius = 0.0001
    lat = orion.location.global_relative_frame.lat
    lon = orion.location.global_relative_frame.lon

    while True:
        if orion.location.global_relative_frame.alt >= alt * 0.95:
            break
        time.sleep(1)

    points = 30        # points in the circle we draw
    loops = 2 * points # do two loop
    coords = []
    for i in range(0, loops):
        degrees = (i/points)*360
        radians = (math.pi/180)*degrees
        x = lat + radius * math.cos(radians)
        y = (lon + radius * math.sin(radians))
        coords.append((x,y))

    cmds = orion.commands
    cmds.clear()

    for lat,lon in coords:
        point = LocationGlobalRelative(lat,lon,alt)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

    cmds.upload()

    orion.mode = VehicleMode("AUTO")
    time.sleep(1)

    while True:
        nextwaypoint=orion.commands.next
        time.sleep(1)



##### RUNNER ######

# setup
orion = connect('tcp:127.0.0.1:5762', wait_ready=True)

drone = Drone(orion, sim=False)
drone.airspeed = 5

# takeoff
drone.takeoff(10)

detector = live_ai(capture_index=0, model_name='yolov5s')
detector()
