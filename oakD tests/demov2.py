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
    #controller = serial.Serial(port='/dev/ttyACM1',baudrate=115200, timeout=1)
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
prev_label = curr_label = label = ""
flag_m1 = flag_m2 = 0
flag_p = flag_p1 = flag_p2 = flag_p3 = 0
dist_counter = 0 # to prevent misdetections 9hopefully)
wave_freq = 1 # Hz
autonomous_mode = 0 # 1=wellcome, 2=withdraw

distance_active = 600   #mm
distance_wave = 1500    #mm

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

def STATE9(): #STATE6()
    write_dxl('9\n')

def pneu_order(tentacles,pressure):
    for tentacle in tentacles: write_pneu(tentacle+str(pressure))
pressure_inflation = 1500; pressure_atm = 990;

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
    # #Default
    # write_pneu('Z');                        time.sleep(5)
    # # Pull all
    # write_dxl('7\n');                       time.sleep(5)
    # # Release
    # pneu_order("ABC",pressure_inflation);   time.sleep(4)
    # write_dxl('8\n');                       time.sleep(12)
    # write_pneu('Z')

################################################################

#Controller 
    #Controller  1 = all inflation
    #Controller  2 = all deflation
    #Controller  3 = LEFT  Pull - Release 
    #Controller  4 = RIGHT Pull - Release
    #Controller  5 = all Pull
    #Controller  6 = all Release
    #Controller  7 = wave condition 1 - speed change - wave(1/2/3) 
    #Controller  8 = wave condition 2 - different phase
    #Controller  9 = enter autonomous wellcome 
    #Controller 10 = enter autonomous withdraw

    STATE = 0
    COMMAND = 11
    PREV_COMMAND = 0
    print('READY')
    while True:
        if controller.in_waiting:
            COMMAND = read_control()
        #PRESSURES = pneumatic.readline()
        #elif COMMAND!=10:
        #    COMMAND=11

        # if COMMAND == PREV_COMMAND:
        #     print('Same COMMAND '+str(COMMAND)+' '+str(PREV_COMMAND))
        if COMMAND != PREV_COMMAND:
            PREV_COMMAND = COMMAND 
            if COMMAND == 1:
                print('Controller  1 = all inflation')
                pneu_order("ABC",pressure_inflation);
            
            if COMMAND == 2:
                print('Controller  2 = all deflation')
                pneu_order("ABC",pressure_atm);

            if COMMAND == 3:
                print('Controller  3 = LEFT  Pull - Release')
                write_dxl('1\n');                       time.sleep(5)
                pneu_order("A",pressure_inflation);     time.sleep(3)
                write_dxl('3\n');                       time.sleep(10)

            if COMMAND == 4:
                print('Controller  4 = RIGHT Pull - Release')
                write_dxl('2\n');                       time.sleep(5)
                pneu_order("B",pressure_inflation);     time.sleep(3)
                write_dxl('4\n');                       time.sleep(10)

            if COMMAND == 5:
                print('Controller  5 = all Pull')    
                write_dxl('7\n'); flag_m1=flag_m2=1;                       

            if COMMAND == 6:
                print('Controller  6 = all Release')
                pneu_order("ABC",pressure_inflation);   time.sleep(3)
                write_dxl('8\n'); flag_m1=flag_m2=0;    time.sleep(10)

            if COMMAND == 7:
                print('Controller  7 = wave condition 1 - speed change - wave(1/2/3)')
                wave_freq +=1
                if wave_freq>3: wave_freq = 1
                write_dxl(str(wave_freq)+'9\n')

            if COMMAND == 8:
                print('Controller  8 = wave condition 2 - different phase')
                write_dxl('59\n')

            if COMMAND == 9: # autonomous wellcome   
                print('Controller  9 = enter autonomous wellcome')
                autonomous_mode = 1

            if COMMAND == 10: # autonomous withdraw
                print('Controller 10 = enter autonomous withdraw')
                autonomous_mode = 2 
            if COMMAND ==11:
                print('Controller 11 = back to manual mode')
                autonomous_mode = 0

# END COMMAND ##################################################

# AUTONOMOUS MODE ##############################################
        if autonomous_mode>0:
            #Controller set to AUTO mode
            #write_control('11\n')

            # CAMERA ###########################################
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
            height = frame.shape[0];    width  = frame.shape[1]

            #enters if detects something
            for detection in detections:

                # Denormalize bounding box
                x1 = int(detection.xmin * width);   x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height);  y2 = int(detection.ymax * height)
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
            # END CAMERA #######################################
                          
            #checks if only    
            label = " "
            
            # Device operation #################################
            # distance_active - distance_wave;  wave mode
            # 0 - distance_active;              active mode (wellcoming/withdrawing)
                # Set Wellcome(1)/Withdrawal(2) modes; using controller
                    # Wellcome(1)
                        # Not pressure
                        # LEFT and RIGHT responded to detected people
                    # Withdrawal(2)
                        # No people detection;  pressurise and partially/fully pull
                        # People detection;     pressurise and release            

            # distance_active - distance_wave;  wave mode
            if min_depth<distance_wave:#1000mm=1m
                print("wave")
                # write_dxl('29\n') # wave at 2 Hz

            # 0 - distance_active;              active mode (wellcoming/withdrawing)
            elif min_depth<distance_active:#1000mm=1m    
                if autonomous_mode == 1: # wellcome
                    print("active - wellcome")
                    # Not pressure
                    # LEFT and RIGHT responded to detected people
                    if pcount >0: # and mindepthcount>3:
                        print('PEOPLE DETECTED!')
                        # mindepthcount = 0

                        # Detect LEFT + LEFT is down
                        if ind[1]<280 and flag_m1!=1: 
                            print('LEFT up: dep '+ str(min_depth) + ' - idx ' + str(ind[1]))
                            write_control('1\n')
                            write_dxl('1\n'); flag_m1 = 1; flag_m2 = 0
                        #elif ind[1]<280 and flag_m1==1 and flag_m2==1:
                        #   print('right was up, releasing right')
                        #   write_dxl('4\n') # release right
                        #   flag_m2=0

                        # Detect RIGHT + RIGHT is down   
                        if ind[1]>320 and flag_m2!=1: 
                            print('RIGHT up: dep '+ str(min_depth) + ' - idx ' + str(ind[1]))
                            write_control('2\n')
                            write_dxl('2\n'); flag_m2 = 1; flag_m1 = 0
                        # elif ind[1]>320 and flag_m2==1 and flag_m1==1: # Detect right
                            #  print('left was up, releasing left')
                            #  write_dxl('3\n') # release left
                            # flag_m1=0

                    # nothing in distance_active = release all
                    if flag_m1!=0 or flag_m2!=0:
                        write_control('8\n')
                        print('NOTHING in distance_active, release all')
                        pneu_order("ABC",pressure_inflation); flag_p=1; time.sleep(3)
                        write_dxl('8\n'); flag_m1=flag_m2=0; time.sleep(5)
            
                if autonomous_mode == 2: # withdraw
                    print("active - withdraw")               
                    # No people detection;  pressurise and partially/fully pull
                    # People detection;     pressurise and release    
                    pneu_order("ABC",pressure_inflation); flag_p=1; time.sleep(3)
                    if pcount >0: # and mindepthcount>3:
                        print('PEOPLE!')
                        write_dxl('8\n'); flag_m1=flag_m2=0; time.sleep(5)
                    elif pcount ==0:
                        print('NO PEOPLE!')
                        write_dxl('7\n'); flag_m1=flag_m2=1; time.sleep(5)    

        pcount=0    

        # To Exit ##########################################      
        if cv2.waitKey(1) == ord('q'):
            break