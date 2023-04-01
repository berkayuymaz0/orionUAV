#imports
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative,Command,mavutil
import time
import math
import argparse
import torch
import numpy as np
import cv2
from threading import Thread
#connect to the drone
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

fire_detected = False
house_detected = False

#arm and takeoff
def arm_and_takeoff(alt):
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

#function to set the default speed of the drone
def set_speed(speed):
    #speed is used to set the speed of the drone
    vehicle.airspeed = speed
    vehicle.groundspeed = speed

#function to change the mode of the drone
def change_mode(mode):
    #mode is used to set the mode of the drone
    vehicle.mode = VehicleMode(mode)
    #while loop is used to check if the drone has changed mode
    while vehicle.mode.name!=mode:
        print("Waiting for mode change")
        #time is used to delay the program
        time.sleep(1)
    #print is used to print the mode of the drone
    print("Mode changed to: ", mode)

#function to change the altitude of the drone
def change_alt(alt):
    #goto is used to set the drone to move to the target location
    goto(0, 0, lambda location: vehicle.simple_goto(location, groundspeed=0.01))
    #while loop is used to check if the drone has reached the target location
    while vehicle.location.global_relative_frame.alt < alt * 0.95:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #time is used to delay the program
        time.sleep(1)
    print("Reached target altitude")

#function to calculate the distance between two points
def get_distance_metres(aLocation1, aLocation2):
    #dlat is used to calculate the difference in latitude
    dlat = aLocation2.lat - aLocation1.lat
    #dlon is used to calculate the difference in longitude
    dlong = aLocation2.lon - aLocation1.lon
    #return is used to return the distance between the two points
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#function to get the location of the drone
def get_location_metres(originalLocation, dNorth, dEast):
    #earthRadius is used to set the radius of the earth
    earthRadius=6378137.0
    #dlat is used to calculate the difference in latitude
    dlat = dNorth/earthRadius
    #dlon is used to calculate the difference in longitude
    dlon = dEast/(earthRadius*math.cos(math.pi*originalLocation.lat/180))
    #newlat is used to calculate the new latitude
    newlat = originalLocation.lat + (dlat * 180/math.pi)
    #newlon is used to calculate the new longitude
    newlon = originalLocation.lon + (dlon * 180/math.pi)
    #if statement is used to check if the new latitude is within the range of -90 to 90
    if newlat > 90:
        newlat = 90
    #if statement is used to check if the new latitude is within the range of -90 to 90
    if newlat < -90:
        newlat = -90
    #return is used to return the new latitude and longitude
    return LocationGlobalRelative(newlat, newlon,originalLocation.alt)

#function to move the drone to a specific location
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    #currentLocation is used to get the current location of the drone
    currentLocation = vehicle.location.global_relative_frame
    #targetLocation is used to set the target location of the drone
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    #targetDistance is used to calculate the distance between the current location and the target location
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    #gotoFunction is used to set the drone to move to the target location
    gotoFunction(targetLocation)
    #while loop is used to check if the drone has reached the target location
    while vehicle.mode.name=="GUIDED":
        #remainingDistance is used to calculate the distance between the current location and the target location
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        #print is used to print the remaining distance
        print("Distance to target: ", remainingDistance)
        #if statement is used to check if the drone has reached the target location
        if remainingDistance<=targetDistance*0.01:
            print("Reached target")
            break
        #time is used to delay the program
        time.sleep(2)

#function to land the drone
def land():
    #change_mode is used to change the mode of the drone to LAND
    change_mode("LAND")
    #while loop is used to check if the drone has landed
    while vehicle.armed:
        print("Waiting for landing")
        #time is used to delay the program
        time.sleep(1)
    #print is used to print that the drone has landed
    print("Landed")

#function to disarm the drone
def disarm():
    #armed is used to disarm the drone
    vehicle.armed = False
    #while loop is used to check if the drone has disarmed
    while vehicle.armed:
        print("Waiting for disarming")
        #time is used to delay the program
        time.sleep(1)
    #print is used to print that the drone has disarmed
    print("Disarmed")

#function to return the drone to the launch location
def return_to_launch():
    #change_mode is used to change the mode of the drone to RTL
    change_mode("RTL")
    #while loop is used to check if the drone has returned to the launch location
    while vehicle.armed:
        print("Waiting for RTL")
        #time is used to delay the program
        time.sleep(1)
    #print is used to print that the drone has returned to the launch location
    print("RTL")

#function to make the drone move in square
def move_square():
    alt= 10
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

#function to make the drone move in circle
def move_sircle():

    alt = 10 
    radius = 0.0001
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon

    while True:
        if vehicle.location.global_relative_frame.alt >= alt * 0.95:
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

    cmds = vehicle.commands
    cmds.clear()

    for lat,lon in coords:
        point = LocationGlobalRelative(lat,lon,alt)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

    cmds.upload()

    vehicle.mode = VehicleMode("AUTO")
    time.sleep(1)

    while True:
        nextwaypoint=vehicle.commands.next
        time.sleep(1)


class live_ai:
    
    def __init__(self, capture_index, model_name):

        self.capture_index = capture_index
        self.model = self.load_model(model_name)
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

    def get_video_capture(self):

        return cv2.VideoCapture(self.capture_index)

    def load_model(self, model_name):

        if model_name:
            model = torch.hub.load('../yolov5', 'custom', path=model_name, source='local')
        else:
            model = torch.hub.load('/yolov5', 'yolov5s', pretrained=True)
        return model

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
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
                if detector.class_to_label(labels[i]) == 'person':
                    fire_detected = True
         
                if detector.class_to_label(labels[i]) == 'house':
                    house_detected = True
                else:
                    house_detected = False
                    fire_detected = False
                    print("No fire and house detected")
         
        return frame

    def __call__(self):
        cap = self.get_video_capture()
        assert cap.isOpened()
      
        while True:
              
            ret, frame = cap.read()
            assert ret
            
            frame = cv2.resize(frame, (512,512))
            
            results = self.score_frame(frame)
            frame = self.plot_boxes(results, frame)


            cv2.imshow('cam', frame)
 
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
      
        cap.release()
        cv2.destroyAllWindows()
        
detector = live_ai(capture_index=0, model_name='last.pt')




arm_and_takeoff(10)
time.sleep(1)

p1 = Thread(target = detector)
p2 = Thread(target = move_square)
p3 = Thread(target = move_sircle)

p1.start()
    
while True:
    
   if fire_detected == True:
       land()
       time.sleep(2)
       p3.start()
       if house_detected == True:
           land()
           time.sleep(2)
           break

      