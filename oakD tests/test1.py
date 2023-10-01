#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import serial


#for serial connection or not
testmode = 1 

if testmode ==0:
    #Establish Serial ports
    
    arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
    #pneumatic = serial.Serial(port='COM15',baudrate=115200, timeout=.1)
    arduino.write(bytes('00', 'utf-8'))

    def write_dxl(eeee):
        arduino.write(bytes(eeee, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data

    def write_pneu(bbbb):
        i = bbbb
        #pneumatic.write(bytes(bbbb, 'utf-8'))
        #time.sleep(0.05)
        #data = arduino.readline()
        #return data
        time.sleep(0.05)
        data = 'PNEUMATIC sent: ' + str(bbbb)
    #data = arduino.readline()

if testmode ==1:
    # no serial connections
    def write_dxl(eeee):
        #arduino.write(bytes(eeee, 'utf-8'))
        time.sleep(0.05)
        data = 'Serial sent: ' + str(eeee)
        return data

    def write_pneu(bbbb):
        i = bbbb
        #pneumatic.write(bytes(bbbb, 'utf-8'))
        time.sleep(0.05)
        #data = arduino.readline()
        data = 'Serial sent: ' + str(bbbb)
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
    flag_m1=1
    flag_m2=1


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
            roiData = detection.boundingBoxMapping
            roi = roiData.roi
            #roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
            topLeft = roi.topLeft()
            bottomRight = roi.bottomRight()
            xmin = int(topLeft.x)
            ymin = int(topLeft.y)
            xmax = int(bottomRight.x)
            ymax = int(bottomRight.y)
            #cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)
            cv2.rectangle(depthFrame,(xmin, ymin), (xmax, ymax), color, 1)
            xmid = (xmax+xmin)/2
            ymid = (ymax+ymin)/2
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            
            #preview stuff - comment out when production
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

        personal_space = 1000

        #DETECT PEOPLE?
        if label == "person":
            if detection.confidence>0.75 and detection.spatialCoordinates.z<personal_space:
                pcount+=1
            if pcount == 1:
                print(str(pcount)+'people detected!')
            if pcount>3:
                print(str(pcount)+'people detected!')
                pcount=0
                print (detection.spatialCoordinates.x, detection.spatialCoordinates.y, detection.spatialCoordinates.z)
                #print(min_depth)
                if detection.spatialCoordinates.x > 5:
                    write_dxl('5')
                    print('LEFT')
                    
                if detection.spatialCoordinates.x < 5:
                    write_dxl('6')
                    print('RIGHT')
                
                persondist = np.round(detection.spatialCoordinates.z)
                if persondist<min_depth:
                    min_depth = persondist
                
                print("they are: " + str(persondist) + "mm away")
                
            #write_dxl(min_depth)
            
            time.sleep(0.1)
            label = " "
            #print(value) # printing the value
        else:
            pcount=0
        
        #OBSTACLE DETECTION
        depth_thresholds =[300, 600, 900, 1200, 1500, 1800]
        #print(min_depth)
        
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
       # if arduino.in_waiting >0:
        #    print(arduino.readline())
        if dist_msg>3:
            pneumsg = 'A1000'
            write_pneu(pneumsg)
        
        #Obstacle inbound
        if dist_msg == 2:
            #print(arduino.readline())
            #arduino.flush()
            
            if ind[1]<280 and flag_m1==1:

                a = write_dxl(str(dist_msg)+'1\n')
                print(a)
                print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  LEFT')
                flag_m1 = -1
 
                    
            elif ind[1]>320 and flag_m2==1:
               
                a = write_dxl(str(dist_msg)+'2\n')
                print (a)
                print('dep '+ str(min_depth) + ' - idx ' + str(ind[1])+'  RIGHT')
                flag_m2 = -1
                
        if flag_m2 == -1 and min_depth>850:
            flag_m2=1
            pneumsg = 'A1100'
            b = write_pneu(pneumsg)
            print(b)
            a = write_dxl(str(dist_msg)+'4\n')
            print(a)
            print('release')



        if flag_m1 == -1 and min_depth>850: # only goes down if scene clears
            flag_m1=1
            pneumsg = 'A1100'
            b = write_pneu(pneumsg)
            print(b)

            a = write_dxl(str(dist_msg)+'3\n')
            print(a)
            print('release')

        if cv2.waitKey(1) == ord('q'):
            break
