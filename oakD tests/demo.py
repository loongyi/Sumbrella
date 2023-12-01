#!/usr/bin/env python3
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import serial

#for serial connection or not
testmode =1 # don't change this; fix at 0

if testmode ==0:
    #Establish Serial ports
    
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
    pneumatic = serial.Serial(port='/dev/ttyS0',baudrate=115200, timeout=.1)
    controller = serial.Serial(port='/dev/ttyACM1',baudrate=115200, timeout=1)

    #arduino.write(bytes('00', 'utf-8'))
    #pneumatic.write(bytes('Z', 'utf-8'))
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
        #time.sleep(0.05)tyhrst
        #data = 'PNEUMATIC sent: ' + str(bbbb)
    #data = arduino.readline()

    def write_control(cccc):
        controller.write(bytes(cccc, 'utf-8'))
        time.sleep(0.05)
        #data = controller.readline()
        #return data
        
if testmode ==1:
    # no serial connections
    controller = serial.Serial(port='COM6',baudrate=115200, timeout=1)
    def write_dxl(eeee):
        #arduino.write(bytes(eeee, 'utf-8'))
        time.sleep(0.05)
        data = 'DXL sent: ' + str(eeee)
        print(data)
        return data

    def write_pneu(bbbb):
        i = bbbb
        #pneumatic.write(bytes(bbbb, 'utf-8'))
        time.sleep(0.05)
        #data = arduino.readline()
        data = 'Pneumatic sent: ' + str(bbbb)
        print(data)
        return data
    
    def write_control(cccc):
        controller.write(bytes(cccc, 'utf-8'))
        time.sleep(0.05)
        data = 'Controller sent: ' + str(cccc)
        print(data)
        return data
    
    def read_control():
        data = controller.readline().rstrip()
        if data:
            dataaa = int(float(data))
            print (dataaa)
            return dataaa
        else:
            return 11
    
    
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

#detectors and flags
pcount = 0
prev_label = ""
curr_label = ""
label = ""
flag_m1=0
flag_m2=0
flag_p =0
flag_p1 =0
flag_p2 =0
flag_p3 =0
dist_counter = 0 # to prevent misdetections 9hopefully)

#params
personal_space = 1000 #mm
mindepthcount = 0

def INFLATE_DEFLATE(tentacle):# 0 for all, 1 for L, 2 for R, 3 for B

    if tentacle == 0 :
        if flag_p==0: # 0=deflated, 1=inflated
            write_pneu('Z') #Z=all deflated
            write_pneu('A1500') #inflate
            flag_p=1
            print('<2000mm, inflated')
            time.sleep(1)
            
        else:
            if flag_p==1:
                write_pneu('Z')
                write_pneu('B1400') # deflate
                print('<2000mm, deflated')
                time.sleep(2)
                write_pneu('Z')                   
                flag_p=0

    if tentacle == 1:
        if flag_p1==0: # 0=deflated, 1=inflated
            write_pneu('Z') #Z=all deflated
            write_pneu('A1500') #inflate
            flag_p1=1
            print('<2000mm, inflated')
            time.sleep(1)
            
        else:
            if flag_p1==1:
                write_pneu('Z')
                write_pneu('B1400') # deflate
                print('<2000mm, deflated')
                time.sleep(2)
                write_pneu('Z')                   
                flag_p1=0

    if tentacle == 2:
        if flag_p2==0: # 0=deflated, 1=inflated
            write_pneu('Z') #Z=all deflated
            write_pneu('A1500') #inflate
            flag_p2=1
            print('<2000mm, inflated')
            time.sleep(1)
            
        else:
            if flag_p2==1:
                write_pneu('Z')
                write_pneu('B1400') # deflate
                print('<2000mm, deflated')
                time.sleep(2)
                write_pneu('Z')                   
                flag_p2=0

    if tentacle == 3:
        if flag_p3==0: # 0=deflated, 1=inflated
            write_pneu('Z') #Z=all deflated
            write_pneu('A1500') #inflate
            flag_p3=1
            print('<2000mm, inflated')
            time.sleep(1)
            
        else:
            if flag_p3==1:
                write_pneu('Z')
                write_pneu('B1400') # deflate
                print('<2000mm, deflated')
                time.sleep(2)
                write_pneu('Z')                   
                flag_p3=0

def STATE0(): # STOP,RESET
    write_dxl('0\n')

def STATE1(): #LEFT UP
    write_dxl('1\n')

def STATE2(): #RIGHT UP
    write_dxl('2\n')

def STATE3(): #LEFT DOWN
    write_dxl('3\n')

def STATE4(): #RIGHT DOWN
    write_dxl('4\n')

def STATE5(): #ASYNC WAVE
    write_dxl('5\n')

def STATE6(): #SYNC WAVE
    write_dxl('6\n')

def STATE7(): #ALL UP
    write_dxl('7\n')

def STATE8(): #ALL RELEASE
    write_dxl('8\n')

def STATE9(): #
    write_dxl('9\n')



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


######################## Editable ##############################
# Pre testing- boot up movements
    write_pneu('Z')
    time.sleep(3)
    write_pneu('A1400')
    write_pneu('B1400')
    write_pneu('C1400')
    time.sleep(4)
    write_pneu('B990')
    
    write_dxl('7\n')
    time.sleep(3)
    
    write_pneu('A1400')
    time.sleep(4)
    
    write_dxl('8\n')
    time.sleep(10)
    write_pneu('Z')
################################################################
    STATE = 0
    COMMAND =11
    print('READY')
    while True:
        if controller.in_waiting:
            COMMAND = read_control()
        #PRESSURES = pneumatic.readline()
        elif COMMAND!=10:
            COMMAND=11

        if COMMAND == 1:
            write_pneu('A1500') #inflate
            write_pneu('B1500') #inflate
            write_pneu('C1500') #inflate
        
        if COMMAND == 2:
            write_pneu('A990') #inflate
            write_pneu('B990') #inflate
            write_pneu('C990') #inflate

        if COMMAND == 3:
            write_dxl('7\n')

        if COMMAND == 4:
            STATE4()

        if COMMAND == 5:
            STATE5()

        if COMMAND == 6:
            STATE6()

        if COMMAND == 7:
            STATE7()

        if COMMAND == 8:
            STATE8()

        if COMMAND == 9:
            STATE6()

        if COMMAND == 10: #Controller set to AUTO mode
            write_control('11\n')
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

            detections = inDet.detections

            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]

            #enters if detects something
            for detection in detections:

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

            #can comment out in production code
            cv2.imshow("preview", frame)

    ######################## Editable ##############################
    #         depth_thresholds =[250, 500, 900, 1200, 1500, 2000]
    # #         print(min_depth)
            
    #         if min_depth < depth_thresholds[0]:
    #             dist_msg = 1
    #         elif min_depth < depth_thresholds[1]:
    #             dist_msg = 2
    #         elif min_depth < depth_thresholds[2]:
    #             dist_msg = 3
    #         elif min_depth < depth_thresholds[3]:
    #             dist_msg = 4
    #         elif min_depth < depth_thresholds[4]:
    #             dist_msg = 5
    #         elif min_depth > depth_thresholds[5]:
    #             dist_msg = 6
    #         else:
    #             dist_msg = 9
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
                        write_control('0\n')
                        #write_pneu('Z') #Z=all deflated
                        write_pneu('A1500') #inflate
                        write_pneu('B1500') #inflate
                        write_pneu('C1500') #inflate
                        flag_p=1
                        print('<2000mm, inflated')
                        time.sleep(1)
                        
                else:
                    if flag_p==1:
                        #write_pneu('Z')
                        write_control('0\n')
                        write_pneu('A990') # deflate
                        write_pneu('B990') # deflate
                        write_pneu('C990') # deflate
                        print('<2000mm, deflated')
                        time.sleep(3)
                        write_pneu('Z')                   
                        flag_p=0
            
            if min_depth<600:#1000mm=1m make this 1200!!!!!!!!!
                # 2.1 person = All pull/release
                mindepthcount+=1
                if pcount==1: # in released state
                    write_control('7\n')
                    print('1 person! pullup all')
                    write_dxl('7\n') # pull all
                    flag_m1=1 # Pulled up
                    flag_m2=1
                elif pcount>=2: # withdrawal
                    write_control('8\n')
                    print('more than 2p, withdraw')
                    #write_pneu('Z')
                    write_pneu('A1500') #inflate
                    write_pneu('B1500') #inflate
                    write_pneu('C1500') #inflate
                    flag_p=1
                    time.sleep(2)
                    write_dxl('8\n') # release all
                    flag_m1=0 # Released
                    flag_m2=0
                    
                # 2.2 object = respond left/right
                elif pcount == 0 and mindepthcount>3:
                    print('OBJECT DETECTED!')
                    mindepthcount = 0
                    if ind[1]<280 and flag_m1!=1: # Detect left
                        print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  LEFT up')
                        write_control('1\n')
                        write_dxl('1\n') # Pull left
                        flag_m1 = 1 # M1 Pulled
                        flag_m2 = 0
                    #elif ind[1]<280 and flag_m1==1 and flag_m2==1:
                    #   print('right was up, releasing right')
                    #   write_dxl('4\n') # release right
                    #   flag_m2=0
                        
                    if ind[1]>320 and flag_m2!=1: # Detect right
                        print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  RIGHT up')
                        write_control('2\n')
                        write_dxl('2\n') # Pull right
                        flag_m2 = 1 # M2 Pulled
                        flag_m1 = 0
                        

                # elif ind[1]>320 and flag_m2==1 and flag_m1==1: # Detect right
                    #  print('left was up, releasing left')
                    #  write_dxl('3\n') # release left
                    # flag_m1=0

                
            # nothing in 1200mmm = release all
            elif flag_m1!=0 or flag_m2!=0:
                write_control('8\n')
                print('nothing <1200, release all')
                #write_pneu('Z')
                write_pneu('A1500') #inflate
                write_pneu('B1500') #inflate
                write_pneu('C1500') #inflate
                flag_p=1
                time.sleep(2)
                write_dxl('8\n') #release all
                flag_m1=0
                flag_m2=0
                
            pcount=0
            #time.sleep(0.25)

################################################################      
        if cv2.waitKey(1) == ord('q'):
            break
