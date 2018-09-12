#!/usr/bin/env python
####################################################################################
# The central ROS node to receive all datastreams
# It receives images from the camera and a camera calibration
# It outputs 3D-position estimates of the detected boats
####################################################################################
import cv2
cv2.useOptimized()
import numpy as np
import time
import rospy
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from image_geometry import cameramodels

from searchwing import roiDetector
from searchwing import roiDescriptor
from searchwing import boatTracker

cvbridge = CvBridge()
#####################
# IO of the ROS-Node
#####################
#Entry point of the programm to receive data through subscribers
#Program is eventtriggered, thus it calls the callback when a new Image is received
def listener():
    rospy.init_node('roiDetector', anonymous=True)

    imgTopicName = 'gopro/image_raw' # subscribe the simulated camera data stream
    calibTopicName = "gopro/camera_info" # subscribe to the fake camera calib data stream
    rospy.Subscriber(imgTopicName,Image,callbackImg)
    rospy.Subscriber(calibTopicName,CameraInfo,callbackCamCalib)

    #Loop until new data arrives
    rospy.spin()

#Callback for the camera calib which is provided from external ros node
camModel = cameramodels.PinholeCameraModel()
def callbackCamCalib(data):
    if camModel.K is None:  # Only init model if not yet initalized
        camModel.fromCameraInfo(data)

#Debugging Publishers
debugPicPub = rospy.Publisher('detectionsPic', Image, queue_size=1)
import sensor_msgs.point_cloud2 as pc2
picProjPub = rospy.Publisher('picProjection',PointCloud2, queue_size=1)

#####################
# Process datastreams
#####################
#Transform functions
import tf
from geometry_msgs.msg import PointStamped
tf_listener = tf.TransformListener() # must be outside callback
tf_transformer = tf.Transformer()
tf_broadcaster = tf.TransformBroadcaster()

#Stuff to load the ROI descriptor
descriptor = roiDescriptor.Descriptors()
descriptorLen = len(descriptor.getDescrNames())
#Stuff to load the classifier
from sklearn.externals import joblib
import rospkg
rospack = rospkg.RosPack()
classifierpath = rospack.get_path('searchwing') + "/config/classifier.pkl"
clf=joblib.load(classifierpath)
clf.n_jobs=1

#Filter ROIs
min3DLen=2  #[m]
max3DLen=35 #[m]
min2DSize=20 #[pix]
max2DSize=150#[pix]


tracker = boatTracker.boatTracker()

#######Helper Functions
# A generic function to compute the intersection of a 3D Line with a 3D Plane
def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
    ndotu = planeNormal.dot(rayDirection)
    if abs(ndotu) < epsilon:
	    raise RuntimeError("no intersection or line is within plane")
    w = rayPoint - planePoint
    si = -planeNormal.dot(w) / ndotu
    Psi = w + si * rayDirection + planePoint
    return Psi

# Function to get the 3D Position of a Pixel on the Water in a Picture by transforming the 2D Pos of the Pixel to a 3D Pos in the World
def getPixel3DPosOnWater(Point2D,Drone3DPosInMap,stamp):
    #get Vector from origin to 3d pos of pixel in the camcoordinatesystem
    #see https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html for visualization
    Point3D = camModel.projectPixelTo3dRay(Point2D)

    #generate point in cam coordinate system in ros
    pt = PointStamped()
    pt_transformed = PointStamped()
    pt.point.x = Point3D[0]
    pt.point.y = Point3D[1]
    pt.point.z = Point3D[2]
    pt.header.frame_id = "cam"
    pt.header.stamp = stamp

    #transform point to drone coordinate system
    pt_transformed = tf_listener.transformPoint("map", pt)

    # Define plane on ground = sea
    planeNormal = np.array([0, 0, 1]) # normal vector points up along z axis
    planePoint = np.array([0, 0, 0])  # Any point on the plane

    # Define ray through pixel in drone coordinate system
    rayPoint = np.array([Drone3DPosInMap.point.x,
                         Drone3DPosInMap.point.y,
                         Drone3DPosInMap.point.z])  # Any point along the ray
    rayDirection=np.array([pt_transformed.point.x-Drone3DPosInMap.point.x,
                           pt_transformed.point.y-Drone3DPosInMap.point.y,
                           pt_transformed.point.z-Drone3DPosInMap.point.z])
    rayPointOnPlane = LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)
    return rayPointOnPlane

#The central callbackfunction
def callbackImg(data):
    picStamp= data.header.stamp #important to get the timestamp of the recording of the image to get the 3d-position of the drone at the time

    startges = time.time()

    #Get Picture from Camera-Node
    cv_img = cvbridge.imgmsg_to_cv2(data, "bgr8")
    rgb=cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)

    #Run ROI-Detection Pipeline
    start = time.time()
    ROIs,contours,mask_img=roiDetector.detectROIs(rgb, gradSize=1, gaussBlurKernelSize=15, openSize=3)
    imgWithBoundingBoxes = roiDetector.drawBoundingBoxesToImg(rgb,ROIs)
    end = time.time()
    print("detectROIs [sek]:",end - start)

    #Get 3D-Position of drone in the World
    start = time.time()
    drone3dPos = PointStamped()
    drone3dPos.point.x = 0
    drone3dPos.point.y = 0
    drone3dPos.point.z = 0
    drone3dPos.header.frame_id = "base_link"
    drone3dPos.header.stamp = picStamp
    tf_listener.waitForTransform("base_link","map",picStamp,rospy.Duration(1)) #wait until the needed transformation is ready, as the image can be provided faster the the telemetry
    drone3dPosInMap=tf_listener.transformPoint("map",drone3dPos)

    #Filtering of ROIs
    contoursSizeFiltered=[]
    ROIsSizeFiltered=[]
    ROIs3dCenters=[]
    # Filter by 2D-Size of ROIs
    for idx, oneContour in enumerate(contours, 0):
        ROIwidth = ROIs[idx][2]-ROIs[idx][0]
        ROIheight = ROIs[idx][3]-ROIs[idx][1]
        if ROIwidth < min2DSize or ROIwidth > max2DSize or ROIheight < min2DSize or ROIheight > max2DSize:
            continue
        pt2dROICenter = np.array([ROIs[idx][0] + (ROIwidth / 2) , ROIs[idx][1] + (ROIheight / 2)], np.int)
        pt3dROICenter = getPixel3DPosOnWater(pt2dROICenter, drone3dPosInMap, picStamp)  # Get 3D Position of
        contoursSizeFiltered.append(oneContour)
        ROIsSizeFiltered.append(ROIs[idx])
        ROIs3dCenters.append(pt3dROICenter)
        # print(oneContour[0],dist3D)
        cv2.circle(imgWithBoundingBoxes, (pt2dROICenter[0], pt2dROICenter[1]), 10, (0, 255, 0), 3)

    """
    #Filter by estimated 3D-Size of Objects in ROIs
    for idx, oneContour in enumerate(contours, 0):
        pt2D1,pt2D2=roiDetectorCV.getExtreme2dPoints(oneContour,debug=False)
        pt3D1 = getPixel3DPosOnWater(pt2D1,drone3dPosInMap,picStamp)
        pt3D2 = getPixel3DPosOnWater(pt2D2,drone3dPosInMap,picStamp)
        pt3DDiff = pt3D1-pt3D2
        objLen3D = np.linalg.norm(pt3DDiff)
        pt3dROICenter = pt3D2 + pt3DDiff*0.5
        if objLen3D > min3DLen and objLen3D < max3DLen:
            contoursSizeFiltered.append(oneContour)
            ROIsSizeFiltered.append(ROIs[idx])
            ROIs3dCenters.append(pt3dROICenter)
            #print(oneContour[0],dist3D)
            cv2.line(imgBB, (pt2D1[0],pt2D1[1]),(pt2D2[0],pt2D2[1]),(0, 255, 0), 3)
    """
    end = time.time()
    print("3d pos estimate [sek]:", end - start)

    #Get cutout images of the ROIs
    start = time.time()
    roiBGRImages = roiDetector.extractImagesFromROIs(ROIsSizeFiltered, cv_img)
    roiMASKImages = roiDetector.extractImagesFromROIs(ROIsSizeFiltered, mask_img)
    end = time.time()
    print("extractImagesFromROIs [sek]:", end - start)

    #Calculate descriptor of each cutout image
    roiDescriptions = np.empty((len(ROIsSizeFiltered), descriptorLen,))
    roiDescriptions[:] = np.nan
    start = time.time()
    i = 0
    for (roiBGR, roiMASK) in zip(roiBGRImages, roiMASKImages):
        description = descriptor.calcDescrROI(roiBGR, roiMASK)
        roiDescriptions[i, :] = description
        i = i + 1
    end = time.time()
    #Filter results on validity
    nonNanIndizes=~np.isnan(roiDescriptions).any(axis=1)
    npROIs=np.asarray(ROIsSizeFiltered)
    roiDescriptions=roiDescriptions[nonNanIndizes]
    validROIs=npROIs[nonNanIndizes]
    print("calcDescrROI [sek]:", end - start)

    #Classify the descriptors of the ROIs
    start = time.time()
    roiClassifications=clf.predict(roiDescriptions)
    end = time.time()
    print("predict [sek]:",end - start)

    #Get only data from "boat" classifications for further processing
    boat3DDetections=[]
    for idx,roiClassification in enumerate(roiClassifications,0):
        if(roiClassification == "boat"):
            cv2.rectangle(imgWithBoundingBoxes, (validROIs[idx][0],validROIs[idx][1]), (validROIs[idx][2],validROIs[idx][3]), (255, 255, 0), 3)
            boat3dPoint= ROIs3dCenters[idx]
            boat3DDetections.append(boat3dPoint)
    """
    #visualize 3d Pos of the current boat detections
    for idx,oneDetection in enumerate(boat3DDetections,0):
        frameName = "d" + str(idx)
        tf_broadcaster.sendTransform(oneDetection,
                         (0.0, 0.0, 0.0, 1.0),
                         picStamp,
                         frameName,
                         "map")
    """

    #Track detections over time to validate them
    start = time.time()
    tracker.addDetections(boat3DDetections)
    trackedBoats=[]
    trackedBoats=tracker.getTrackings(minTrackingCount=5)
    #visualize 3d Pos of valid tracked objects
    for oneTrack in trackedBoats:
        frameName = "TRACK" + str(oneTrack.id) + "c" + str(oneTrack.trackingCount)
        tf_broadcaster.sendTransform(oneTrack.pos,
                         (0.0, 0.0, 0.0, 1.0),
                         picStamp,
                         frameName,
                         "map")

    end = time.time()
    print("track boats[sek]:",end - start)

    #Create debugging visualization
    start = time.time()
    imgHeight, imgWidth, channels = rgb.shape
    #Create Points which mark the edges of the camerapic in the world (RVIZ)
    #upperleft
    ptul=getPixel3DPosOnWater([1,1],drone3dPosInMap,picStamp)
    #upperright
    ptur=getPixel3DPosOnWater([1,imgHeight],drone3dPosInMap,picStamp)
    #lowerleft
    ptll=getPixel3DPosOnWater([imgWidth,1],drone3dPosInMap,picStamp)
    #lowerright
    ptlr=getPixel3DPosOnWater([imgWidth,imgHeight],drone3dPosInMap,picStamp)
    pixelProjection3DPoints = [ptul,ptur,ptll,ptlr]
    header = Header()
    header.frame_id = "map"
    header.stamp = picStamp
    image3dProjection = PointCloud2()
    image3dProjection= pc2.create_cloud_xyz32(header,pixelProjection3DPoints)
    picProjPub.publish(image3dProjection)
    end = time.time()
    print("picsize proj estimate [sek]:",end - start)
    rosPic = cvbridge.cv2_to_imgmsg(imgWithBoundingBoxes, "rgb8")
    debugPicPub.publish(rosPic)


    endges = time.time()
    print("================================== sum of all [sek]:",endges - startges)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

