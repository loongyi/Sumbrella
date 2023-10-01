#!/usr/bin/env python3
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import serial

#for serial connection or not
testmode =0 # don't change this; fix at 0

if testmode ==0:
    #Establish Serial ports
    
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
    pneumatic = serial.Serial(port='/dev/ttyS0',baudrate=115200, timeout=.1)
    arduino.write(bytes('00', 'utf-8'))
    pneumatic.write(bytes('Z', 'utf-8'))
    def write_dxl(eeee):
        arduino.write(bytes(eeee, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data

    def write_pneu(bbbb):
        i = bbbb
        pneumatic.write(bytes(bbbb, 'utf-8'))
        time.sleep(0.05)
        data = pneumatic.readline()
        return data
        #time.sleep(0.05)
        #data = 'PNEUMATIC sent: ' + str(bbbb)
    #data = arduino.readline()

if testmode ==1:
    # no serial connections
    def write_dxl(eeee):
        #arduino.write(bytes(eeee, 'utf-8'))
        time.sleep(0.05)
        data = 'Serial sent: ' + str(eeee)
        print(data)
        return data

    def write_pneu(bbbb):
        i = bbbb
        #pneumatic.write(bytes(bbbb, 'utf-8'))
        time.sleep(0.05)
        #data = arduino.readline()
        data = 'Serial sent: ' + str(bbbb)
        print(data)
        return data
    

# Get argument (trained NN model) first
nnBlobPath = str((Path(__file__).parent / Path('mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setSubpixel(True)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    #detectors and flags
    pcount = 0
    prev_label = ""
    curr_label = ""
    label = ""
    flag_m1=0
    flag_m2=0
    flag_p =0
    dist_counter = 0 # to prevent misdetections 9hopefully)
    
    #params
    personal_space = 1000 #mm

######################## Editable ##############################
# Pre testing
    write_pneu('Z')
    write_pneu('B1400')
    time.sleep(5)
    write_pneu('Z')
    
    write_dxl('7\n')
    time.sleep(5)
    
    write_pneu('A1400')
    time.sleep(5)
    
    write_dxl('8\n')
    time.sleep(12)
    write_pneu('Z')
################################################################

    while True:
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = inPreview.getCvFrame()

        depthFrame = depth.getFrame() # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
       
        depthABS=abs(depth_downscaled-min_depth)
        ind = np.unravel_index(np.argmin(depthABS,axis=None),depthABS.shape)
        # percidx = depth_downscaled[np.percentile(depth_downscaled, 1, interpolation='nearest')]
        #max_depth = np.percentile(depth_downscaled, 99)
        #depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        #depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]

        #enters if detects stuff
        for detection in detections:
            #roiData = detection.boundingBoxMapping
            #roi = roiData.roi
            #roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
            #topLeft = roi.topLeft()
            #bottomRight = roi.bottomRight()
            #xmin = int(topLeft.x)
            #ymin = int(topLeft.y)
            #xmax = int(bottomRight.x)
            #ymax = int(bottomRight.y)
            #cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)
            #cv2.rectangle(depthFrame,(xmin, ymin), (xmax, ymax), color, 1)
            #xmid = (xmax+xmin)/2
            #ymid = (ymax+ymin)/2
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
                
            #CHECK IF PEOPLE IN FRAME
            if label == "person" and detection.confidence>0.88 and detection.spatialCoordinates.z<5000:
                pcount+=1
                persondist = int(np.round(detection.spatialCoordinates.z))
                if persondist<min_depth:
                    min_depth = persondist

            #preview stuff - comment out for real test
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)


        #cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
        #cv2.imshow("depth", depthFrameColor)
        
        #can comment out in production code
        cv2.imshow("preview", frame)

######################## Editable ##############################
        depth_thresholds =[250, 500, 900, 1200, 1500, 2000]
#         print(min_depth)
        
        if min_depth < depth_thresholds[0]:
            dist_msg = 1
        elif min_depth < depth_thresholds[1]:
            dist_msg = 2
        elif min_depth < depth_thresholds[2]:
            dist_msg = 3
        elif min_depth < depth_thresholds[3]:
            dist_msg = 4
        elif min_depth < depth_thresholds[4]:
            dist_msg = 5
        elif min_depth > depth_thresholds[5]:
            dist_msg = 6
        else:
            dist_msg = 9
################################################################
            
        #checks if only    
        label = " "
        
######################## Editable ##############################
#STATES
#         if pcount != 0:
#            if detection.spatialCoordinates.z > 2000:
#                write_dxl('9\n') # wave
            
        # 1. If person = pressurised wihtin 2000mm
        if min_depth<2000:#1000mm=1m
            print(str(pcount)+'people detected!  '+str(min_depth)+' - idx '+str(ind[1]))
            if pcount >= 1: # detect person (pcount) more than 1
                if flag_p==0: # 0=deflated, 1=inflated
                    write_pneu('Z') #Z=all deflated
                    write_pneu('A1500') #inflate
                    flag_p=1
                    time.sleep(3)
            else:
                if flag_p==1:
                    write_pneu('Z')
                    write_pneu('B1400') # deflate
                    time.sleep(5)
                    write_pneu('Z')
                    flag_p=0
            
        if min_depth<1200:#1000mm=1m
            # 2.1 person = All pull/release
            if pcount>=1 and flag_m1!=1: # in released state
                write_dxl('7\n') # pull all
                flag_m1=1 # Pulled up
            elif pcount>=2: # withdrawal
                write_pneu('Z')
                write_pneu('A1500') #inflate
                flag_p=1
                time.sleep(5)
                write_dxl('8\n') # release all
                flag_m1=0 # Released
                time.sleep(7.5)
                
            # 2.2 object = respond left/right
            elif pcount == 0:
                if ind[1]<280 and flag_m1!=1: # Detect left
                    write_dxl('1\n') # Pull left
                    flag_m1 = 1 # M1 Pulled
                    if flag_m2==1: #M2 Pulled
                        flag_m1=0
                    print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  LEFT')
                    
                if ind[1]>320 and flag_m2!=1: # Detect right
                    write_dxl('2\n') # Pull right
                    flag_m2 = 1
                    if flag_m1==1:
                        flag_m2=0
                    print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  RIGHT')
            
        # nothing in 1000m = release all
        elif flag_m1!=0 or flag_m2!=0:
            write_pneu('Z')
            write_pneu('A1500') #inflate
            flag_p=1
            time.sleep(5)
            write_dxl('8\n') #release all
            flag_m1=0
            
        pcount=0

################################################################
        
        if cv2.waitKey(1) == ord('q'):
            break
