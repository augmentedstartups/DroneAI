#Created by Luxonis
#Modified by Augmented Startups - 18/12/2020
#Watch the tutorial Series here : http://bit.ly/OAKSeriesPlaylist
from __future__ import division
import json
import socketserver
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
from pathlib import Path
from socketserver import ThreadingMixIn
from time import sleep
import depthai
import cv2
import numpy as np
from PIL import Image


import time

TO_Count = 0
LD_Count = 0
TakeOff_Flag = 0
Landing_Flag = 0
Threshold_Trigger = 10
Altitude = 0



#Initialize variables
det_classes = ["Follow", "Follow", "Follow", "Land", "Follow",
            "Null", "TakeOff", "TakeOff", "Follow", "I", "J", "K",
            "L", "M", "N", "O", "P", "Q", "R", "S",
            "T", "U", "V", "W", "blank", "blank", "blank", "blank",
            "blank", "blank", "blank" ] 


####YAW#Parameters######
posX = 300
speedX = 1
ThresholdX = 75
Red_FLAG = 0
#########################

device = depthai.Device('', False)
#nn2depth = device.get_nn_to_depth_bbox_mapping()

pipeline = device.create_pipeline(config={
    "streams": ["metaout", "previewout"],
    "ai": {
        "blob_file": str(Path('./mobilenet-ssd/modelv4.blob').resolve().absolute()),
        "blob_file_config": str(Path('./mobilenet-ssd/config.json').resolve().absolute())
    }
})

if pipeline is None:
    raise RuntimeError("Error initializing pipelne")

detections = []
print(det_classes)

while True:
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
            #frame = cv2.flip( frame,  0)

            img_h = frame.shape[0]
            img_w = frame.shape[1]

            for detection in detections:
                
                    left, top = int(detection.x_min * img_w), int(detection.y_min * img_h)
                    right, bottom = int(detection.x_max * img_w), int(detection.y_max * img_h)
                 
                    label = "{}".format(det_classes[detection.label])
                    print(label)
                    cv2.putText(frame, label, (left, top - 11), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255,0), 5)
                    
                    ##### Finding BBOX center ####
                    cx = int(left + (right-left)/2)
                    cy = int(top + (bottom-top)/2)
                    # Draw Line
                    ImageCenterX, ImageCenterY = int(img_w/2), int(img_h/2) #Center coordinates
                    line_thickness = 2
                    
                    if (Red_FLAG == 0):
                        cv2.line(frame, (ImageCenterX,ImageCenterY ), (cx, cy), (0, 255, 0), thickness=line_thickness)
                    elif (Red_FLAG == 1):   
                        cv2.line(frame, (ImageCenterX,ImageCenterY ), (cx, cy), (0, 0, 255), thickness=line_thickness)    
                    
                    
                    
                    ## TakeOff Sequence
                    if (Altitude == 0):
                        #print (Altitude, "Altitude _EnTER_takeoff")
                        if (label == "TakeOff"):
                            TO_Count = TO_Count + 1
                            if (TO_Count >=Threshold_Trigger ):
                                TO_Count = 0
                                TakeOff_Flag = 1
                                print (TakeOff_Flag, "TakeOff_Flag")
                                Altitude = 1
                                TakeOff_Flag = 0
                                print (Altitude, "Altitude")
                    
                    ## Landing Sequence
                    elif (Altitude >= 0):  ##Change back
                        #print (Altitude, "AltitudeENTERLANDING")
                        if (label == "Land"):
                            LD_Count = LD_Count + 1
                            if (LD_Count >=Threshold_Trigger ):
                                LD_Count = 0
                                Land_Flag = 1
                                print (Land_Flag, "Land_Flag")
                                Altitude = 0
                                Land_Flag = 0
                                print (Altitude, "Altitude")
                                
                   ############YAW#Rotation#Follow####################
                        ### Vertical
                    if (label == "Follow"):
                        print("FOLLOW Sequence entered")
                        
                        ###Threshold#Indication#######
                        if abs(cx-ImageCenterX)>ThresholdX: 
                            Red_FLAG = 1
                        else:
                            Red_FLAG = 0
                            
                        ###Horizontal#YAW#Shift#######
                        if cx>ImageCenterX +ThresholdX: # RIGHT
                            print("Right")
                            
                        elif cx<ImageCenterX-ThresholdX: # LEFT
                            print("Left")
                                
   
                                    
                        
            
                              




#             server_TCP.datatosend = json.dumps([detection.get_dict() for detection in detections])
#             server_HTTP.frametosend = frame
           
            cv2.imshow('previewout', frame)

    if cv2.waitKey(1) == ord('q'):
        break

del pipeline
del device