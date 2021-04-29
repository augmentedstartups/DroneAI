#Working Augmented Startups v1.3 Takeoff and Landing
from __future__ import division
import time
import os
import platform
import sys

###OAK##
import json
import socketserver
import threading
from io import BytesIO
from pathlib import Path
from socketserver import ThreadingMixIn
from time import sleep
import depthai
import cv2
import numpy as np
from PIL import Image


import time
###

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
#############################




#########################################

########################################



manualArm=False ##If True, arming from RC controller, If False, arming from this script.
targetAltitude=0
if len(sys.argv)>1:
    targetAltitude=float(sys.argv[1])
else:
    targetAltitude=2

############DRONEKIT#################
vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)
# vehicle is an instance of the Vehicle class
print("Autopilot Firmware version: %s" %vehicle.version)
print("Autopilot capabilities (supports ftp): %s" %vehicle.capabilities.ftp)
print("Global Location: %s" %vehicle.location.global_frame)
print("Global Location (relative altitude): %s" %vehicle.location.global_relative_frame)
print("Local Location: %s" %vehicle.location.local_frame)    #NED
print("Attitude: %s" %vehicle.attitude)
print("Velocity: %s" %vehicle.velocity)
print("GPS: %s" %vehicle.gps_0)
print("Groundspeed: %s" %vehicle.groundspeed)
print("Airspeed: %s" %vehicle.airspeed)
print("Gimbal status: %s" %vehicle.gimbal)
print("Battery: %s" %vehicle.battery)
print("EKF OK?: %s" %vehicle.ekf_ok)
print("Last Heartbeat: %s" %vehicle.last_heartbeat)
print("Rangefinder: %s" %vehicle.rangefinder)
print("Rangefinder distance: %s" %vehicle.rangefinder.distance)
print("Rangefinder voltage: %s" %vehicle.rangefinder.voltage)
print("Heading: %s" %vehicle.heading)
print("Is Armable?: %s" %vehicle.is_armable)
print("System status: %s" %vehicle.system_status.state)
print("Mode: %s" %vehicle.mode.name)    # settable
print("Armed: %s" %vehicle.armed)    # settable
vehicle.home_location=vehicle.location.global_frame
print(" Home Location: %s" % vehicle.home_location)
vehicle.parameters['PLND_ENABLED']=0
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30

#########FUNCTIONS###########
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
        print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
        print("Vehicle now in GUIDED MODE. Have fun!!")

        if manualArm==False: 
            vehicle.armed = True
            while vehicle.armed==False:
                print("Waiting for vehicle to become armed.")
                time.sleep(1)
        else:
            if vehicle.armed == False:
                print("Exiting script. manualArm set to True but vehicle not armed.")
                print("Set manualArm to True if desiring script to arm the drone.")
                return None
        print("Look out! Props are spinning!!")

    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
        print("Current Altitude: %f" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

    return None

##Send velocity command to drone
def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

########CONTROL DRONE
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
    
    
def OAKDetection():
    ### OAK Initialize variables############
    det_classes = ["Follow", "Follow", "TakeOff_hand", "Land", "Follow_Hand",
                    "Null", "TakeOff", "Land2","blank", "blank", "blank", "blank",
                    "blank", "blank", "blank" ] 
    TO_Count = 0
    LD_Count = 0
    Flag = 0
    Landing_Flag = 0
    Threshold_Trigger = 20
    Altitude = 0

    device = depthai.Device('', False)
    #nn2depth = device.get_nn_to_depth_bbox_mapping()

    pipeline = device.create_pipeline(config={
        "streams": ["metaout", "previewout"],
        "ai": {
            "blob_file": str(Path('./mobilenet-ssd/model.blob').resolve().absolute()),
            "blob_file_config": str(Path('./mobilenet-ssd/config.json').resolve().absolute())
        }
    })

    if pipeline is None:
        raise RuntimeError("Error initializing pipelne")

    detections = []
    print(det_classes)
    #arm_and_takeoff(targetAltitude)

    while Flag == 0:
        nnet_packets, data_packets = pipeline.get_available_nnet_and_data_packets()
        


        for nnet_packet in nnet_packets:
            detections = list(nnet_packet.getDetectedObjects())

        for packet in data_packets:
            if packet.stream_name == 'previewout':
                data = packet.getData()
                data0 = data[0, :, :]
                data1 = data[1, :, :]
                data2 = data[2, :, :]
                frame = cv2.merge([data0, data1, data2])

                img_h = frame.shape[0]
                img_w = frame.shape[1]

                for detection in detections:
                    
                        left, top = int(detection.x_min * img_w), int(detection.y_min * img_h)
                        right, bottom = int(detection.x_max * img_w), int(detection.y_max * img_h)
                        #print(detection.label)
                        label = "{}".format(det_classes[detection.label])
                        print(label)
                        cv2.putText(frame, label, (left, top - 11), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255,0), 5)
                        
                                ## TakeOff Sequence
                        #if (vehicle.location.global_relative_frame.alt <= 0):
                            #print (Altitude, "Altitude _EnTER_takeoff")
                        if (label == "TakeOff"):
                            #print("ENTERED TAKEOFF ZONE")
                            TO_Count += 1
                            if (TO_Count >=Threshold_Trigger ):
                                TO_Count = 0
                                print (1, "ENTERED TAKEOFF ZONE")
                                #arm_and_takeoff(targetAltitude)
                                Flag = 1
                                
                                                        

                        
                                    
                        
                        ## Landing Sequence
#                         if (vehicle.location.global_relative_frame.alt >= 0.2):
#                             #print (Altitude, "AltitudeENTERLANDING")
#                             if (label == "Land"):
#                                 LD_Count = LD_Count + 1
#                                 if (LD_Count >=Threshold_Trigger ):
#                                     LD_Count = 0
#                                     Land_Flag = 1
#                                     print (Land_Flag, "Land_Flag")
#                                     break
                        
                        
                        
                        

                cv2.imshow('previewout', frame)
        
            
        if cv2.waitKey(1) == ord('q'):
            break

    del pipeline
    del device




if __name__=='__main__':
    try:
        
        
        #
        OAKDetection()
        time.sleep(1)
        print("LIFT OFF")
        arm_and_takeoff(targetAltitude)
        time.sleep(1)
        OAKDetection()
#         send_local_ned_velocity(0,1,0)
#         print("Positive Velocity Commands")
#         time.sleep(1)
#         send_local_ned_velocity(0,-1,0)
#         print("Negative Velocity Commands")
#         time.sleep(1)
#         send_global_velocity(0.5,0,0,1)
#         print("Direction:NORTH Relative to heading of drone")
#         time.sleep(1)
#         send_global_velocity(-0.5,0,0,1)
#         time.sleep(2)
#         print("Direction:SOUTH Relative to heading of drone")
#         condition_yaw(90,0)
#         time.sleep(2)
#         print("Non Relative YAW is TRUE 0")
#         condition_yaw(0,0)
#         time.sleep(2)
#         print("Non Relative YAW is TRUE 0 2")
#         condition_yaw(275,0)
#         time.sleep(2)
#         print("Non Relative YAW is TRUE 180 3")
#         condition_yaw(90,0)
#         time.sleep(2)
#         print("Non Relative YAW is TRUE 180 3")
#         condition_yaw(0,0)
        time.sleep(1)

        print("Preparing for landing")
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode!='LAND':
            time.sleep(1)
            print("Waiting for drone to land")
        time.sleep(1)
        print("Eagle has landed")
    except:
        pass