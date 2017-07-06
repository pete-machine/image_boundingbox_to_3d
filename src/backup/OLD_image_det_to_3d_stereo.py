#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from cv_bridge import CvBridge
import message_filters
import cv2
from sensor_msgs.msg import Image, CameraInfo
from boundingbox_msgs.msg import Boundingbox, Boundingboxes, ImageDetections
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from scipy import ndimage
from functions import boundingboxTo3D

rospy.init_node('get_boundingbox_distance', anonymous=False)
nodeName = rospy.get_name()

# Estimate distance type
paramEstDistanceMethod = rospy.get_param(nodeName+'/estDistanceMethod', 1) # 0: mean, 1: median, 2: 10%-percentile.

# Parameter to specify if bounding boxes are visualized in image. 
paramVisualizeBoundingboxes = rospy.get_param(nodeName+'/visualizeBoundingboxes', 'False') 


# Name of input topics from launch-file. 
topicCamImg = rospy.get_param(nodeName+'/topicCamImage', nodeName+'UnknownInputTopic')
topicCamInfo = rospy.get_param(nodeName+'/topicCamInfo', nodeName+'UnknownInputTopic')
topicDepth = rospy.get_param(nodeName+'/topicDepth', nodeName+'UnknownInputTopic')
#topicBBoxIn = rospy.get_param(nodeName+'/topicBBoxIn', nodeName+'UnknownInputTopic')
topicDetImageInMultiClass = rospy.get_param(nodeName+'/topicDetImageInMultiClass', '')
topicDetImageInSingleClass = rospy.get_param(nodeName+'/topicDetImageInSingleClass', '')

if topicDetImageInMultiClass == '' and topicDetImageInSingleClass == '':
    raise NameError('Either topicDetImageIn or topicDetImageIn2D should be defined in the launch script') 
    
targetFrame = rospy.get_param(nodeName+'/targetFrame', nodeName+'UnknownFrameId')
algorithmName = rospy.get_param(nodeName+'/algorithmName', nodeName+'NotDefined')

configFile = rospy.get_param(nodeName+'/config_file', 'cfg/bb2ismExample.cfg')

configData = open(configFile,'r') 
configText = configData.read()
strsClassNumberAndName = [line for idx,line in enumerate(str(configText).split('\n')) if line is not '' and idx is not 0]
classNumberAndName = {}

for strClass in strsClassNumberAndName:
    strNumberAndClass = strClass.split(' ')
    #print 'Class: ',  int(strNumberAndClass[0]), ', ObjectType: ',  strNumberAndClass[1]
    classNumberAndName[strNumberAndClass[1]] = int(strNumberAndClass[0])

# The expectedMinValue and expectedMaxValue 
expectedMaxValue = rospy.get_param(nodeName+'/expected_max_value', 1.0)

threshold = rospy.get_param(nodeName+'/threshold', 0.0)
expectedMinValue = threshold

#threshold = 300.0

# Get subscripers.
image_sub = message_filters.Subscriber(topicCamImg, Image)
info_sub = message_filters.Subscriber(topicCamInfo, CameraInfo)
depth_sub = message_filters.Subscriber(topicDepth, Image)
#bb_sub = message_filters.Subscriber(topicBBoxIn, Boundingboxes)

print "topicDetImageInSingleClass",topicDetImageInSingleClass, "topicDetImageInMultiClass", topicDetImageInMultiClass

if topicDetImageInSingleClass == '':
    print "topicDetImageInMultiClass"
    det_sub3d = message_filters.Subscriber(topicDetImageInMultiClass, ImageDetections)
else:
    print "topicDetImageInSingleClass"
    det_sub2d = message_filters.Subscriber(topicDetImageInSingleClass, Image)

# Name of output topics from launch-file. 
topicBBoxOut = rospy.get_param(nodeName+'/topicBBoxOut', nodeName+'/BBox3d') 
topicVisualizeOut = rospy.get_param(nodeName+'/topicVisualizeOut', nodeName+'/ImageBBox3d')

# Publishers
pub_bb = rospy.Publisher(topicBBoxOut, MarkerArray , queue_size=0)
if paramVisualizeBoundingboxes == True:
	pub_image_visualize = rospy.Publisher(topicVisualizeOut, Image , queue_size=0)

#topicParts = [strPart for strPart in topicBBoxIn.split('/') if strPart is not '']

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

bridge = CvBridge()
    
# Convert to image to bounding boxes. 
def callback_imageMultiClass(image, info, depth, det_image3d):    
    #print "callback_imageMultiClass"
    imgConfidence = bridge.imgmsg_to_cv2(det_image3d.imgConfidence, desired_encoding="passthrough")
    imgClass = bridge.imgmsg_to_cv2(det_image3d.imgClass, desired_encoding="passthrough")
    
    if imgClass.shape[0] == 1 and imgClass.shape[1] == 1: 
        imgClass = imgClass[0,0]*np.ones(imgConfidence.shape,dtype=np.uint8)
    crop = det_image3d.crop
    
    det_to_3d(image, info, depth,imgClass,imgConfidence,crop )

# Convert to image to bounding boxes. 
def callback_imageOneClass(image, info, depth, det_image2D):  
    #print "callback_imageOneClass"
    imgConfidence = bridge.imgmsg_to_cv2(det_image2D, desired_encoding="passthrough")
    classNumber = classNumberAndName[classNumberAndName.keys()[0]]
    imgClass = classNumber*np.ones(imgConfidence.shape)
    crop = [0.0, 1.0, 0.0,1.0]
    
    det_to_3d(image, info, depth,imgClass,imgConfidence,crop )


def det_to_3d(image, info, depth,imgClass,imgConfidence,crop ):
    bounding_boxes = Boundingboxes()    
    
    for className in classNumberAndName.keys():
        classNumber = classNumberAndName[className]
        # Select only regions within class
        bwClass = imgClass==classNumber
        imgConfidenceClass = np.zeros_like(imgConfidence)
        imgConfidenceClass[bwClass] = imgConfidence[bwClass]
        
        #print "NodeName", nodeName,  " np.min(imgConfidenceClass)",np.min(imgConfidenceClass),"np.max(imgConfidenceClass)",np.max(imgConfidenceClass)
        # Threshold on the specific class. 
        bwImg = imgConfidenceClass>expectedMinValue
        
        blobs_labels,n = ndimage.measurements.label(bwImg)
        dim = bwImg.shape
        slicers = ndimage.find_objects(blobs_labels)    
        
        for slicer in slicers:
            
            normalized = (np.array([slicer[0].start, slicer[0].stop,slicer[1].start, slicer[1].stop],dtype=float)/np.array([dim[0],dim[0],dim[1],dim[1]]))
                    
            rowShift = crop[0]
            rowScale = np.diff(crop[0:2])
            colShift = crop[2]
            colScale = np.diff(crop[2:4])
            
            bbNor = np.squeeze(np.array([rowShift,rowShift,colShift,colShift])+normalized*np.array([rowScale,rowScale,colScale,colScale]).T)
            
            tmpBB = Boundingbox()
            tmpBB.x = bbNor[2]
            tmpBB.y = bbNor[0]
            tmpBB.w = np.diff(bbNor[2:])
            tmpBB.h = np.diff(bbNor[:2])
            prob = np.max(imgConfidenceClass[slicer]).astype(np.float)
            tmpNormalized = (prob-expectedMinValue)/(expectedMaxValue-expectedMinValue)
            tmpBB.prob = np.clip(tmpNormalized,0.0,1.0)
            #print "prob", prob, "Normalized", tmpNormalized , "NormalizedClipped", tmpBB.prob 
            tmpBB.objectType = classNumber
            tmpBB.objectName = className
            
            bounding_boxes.boundingboxes.append(tmpBB)
    
    #if pose.orientation.w == 0:
    
    # Bug-fix. To remove the first '/' in frame. E.g. '/Multisensor/blah' --> 'Multisensor/blah' 
    strParts = image.header.frame_id.split('/')
    if strParts[0] is '':
        headFrame = str.join('/',strParts[1:])
    else:
        headFrame = image.header.frame_id
        
    validTranform = True
    try:
        pose = Pose()
        trans = tfBuffer.lookup_transform( headFrame,targetFrame, rospy.Time())
        #trans = tfBuffer.lookup_transform( 'Multisense/left_camera_optical_frame','velodyne', rospy.Time())
        pose.orientation = trans.transform.rotation
        #print("pose.orientation:",pose.orientation)
        
    except Exception as e: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("WARNING: In image_det_to_3d_stereo node. No transform was found. 2d detection is NOT converted to 3d detection. A markerArray is not published",e.message,e.args)
        pose.orientation.w = 1
        validTranform = False
    
    if validTranform: 
        markerArray, cv_image = boundingboxTo3D(image, info, depth, bounding_boxes,pose,algorithmName,paramVisualizeBoundingboxes,paramEstDistanceMethod)
        pub_bb.publish(markerArray)
    
    if paramVisualizeBoundingboxes:
        imgDim = np.array([cv_image.shape[0],cv_image.shape[0],cv_image.shape[1],cv_image.shape[1]])
        imgCrop = (imgDim*crop).astype(int)
        cv2.rectangle(cv_image,(imgCrop[2],imgCrop[0]),(imgCrop[3],imgCrop[1]),[0,0,0],3)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        pub_image_visualize.publish(image_message)
    
if topicDetImageInSingleClass == '':
    ts_image = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, det_sub3d], 10)
    ts_image.registerCallback(callback_imageMultiClass)
else:
    ts_image2 = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, det_sub2d], 10)
    ts_image2.registerCallback(callback_imageOneClass)

# main
def main():
    print ''
    print 'image_det_to_3d_stereo (', nodeName, ') is subscriping to topics: ', topicCamImg, ' - ' ,topicCamInfo, ' - ' ,topicDepth, ' - ' ,topicDepth
    print 'image_det_to_3d_stereo (', nodeName, ') is publishing topic: ', topicBBoxOut
        
    rospy.spin()

if __name__ == '__main__':
    main()
