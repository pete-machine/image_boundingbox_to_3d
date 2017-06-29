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
topicDetImageIn = rospy.get_param(nodeName+'/topicDetImageIn', nodeName+'UnknownInputTopic')

baseFrameId = rospy.get_param(nodeName+'/baseFrameId', nodeName+'UnknownFrameId')
algorithmName = rospy.get_param(nodeName+'/algorithmName', nodeName+'NotDefined')

configFile = rospy.get_param(nodeName+'/config_file', 'cfg/bb2ismExample.cfg')

configData = open(configFile,'r') 
configText = configData.read()
strsClassNumberAndName = [line for idx,line in enumerate(str(configText).split('\n')) if line is not '' and idx is not 0]
pubOutputTopics = {}

for strClass in strsClassNumberAndName:
    strNumberAndClass = strClass.split(' ')
    print 'Class: ',  int(strNumberAndClass[0]), ', ObjectType: ',  strNumberAndClass[1]

# The expectedMinValue and expectedMaxValue 
expectedMaxValue = rospy.get_param(nodeName+'/expected_max_value', 1.0)

threshold = rospy.get_param(nodeName+'/threshold', 700.0)
expectedMinValue = threshold

#threshold = 300.0

# Get subscripers.
image_sub = message_filters.Subscriber(topicCamImg, Image)
info_sub = message_filters.Subscriber(topicCamInfo, CameraInfo)
depth_sub = message_filters.Subscriber(topicDepth, Image)
#bb_sub = message_filters.Subscriber(topicBBoxIn, Boundingboxes)

det_sub = message_filters.Subscriber(topicDetImageIn, ImageDetections)

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
def callback_image(image, info, depth, det_image):    
    imgConfidence = bridge.imgmsg_to_cv2(det_image.imgConfidence, desired_encoding="passthrough")
    imgClass = bridge.imgmsg_to_cv2(det_image.imgClass, desired_encoding="passthrough")
    crop = det_image.crop    

    bounding_boxes = Boundingboxes()    
    
    
    bwImg = imgConfidence>threshold
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
        prob = np.max(imgConfidence[slicer]).astype(np.float)
        tmpNormalized = (prob-expectedMinValue)/(expectedMaxValue-expectedMinValue)
        tmpBB.prob = np.clip(tmpNormalized,0.0,1.0)
        print "prob", prob, "Normalized", tmpNormalized , "NormalizedClipped", tmpBB.prob 
        tmpBB.objectType = np.median(imgClass[slicer])
        tmpBB.objectName = 'anomaly'
        
        bounding_boxes.boundingboxes.append(tmpBB)

    
    
    
    #if pose.orientation.w == 0:
    pose = Pose()
    try:
        # Bug-fix. To remove the first '/' in frame. E.g. '/Multisensor/blah' --> 'Multisensor/blah' 
        strParts = image.header.frame_id.split('/')
        if strParts[0] is '':
            headFrame = str.join('/',strParts[1:])
        else:
            headFrame = image.header.frame_id

        trans = tfBuffer.lookup_transform( headFrame,baseFrameId, rospy.Time())
        #trans = tfBuffer.lookup_transform( 'Multisense/left_camera_optical_frame','velodyne', rospy.Time())
        pose.orientation = trans.transform.rotation
        #print("pose.orientation:",pose.orientation)
    except Exception as e: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #print("Except",e.message,e.args)
        pose.orientation.w = 1
    
    markerArray, cv_image = boundingboxTo3D(image, info, depth, bounding_boxes,pose,algorithmName,paramVisualizeBoundingboxes,paramEstDistanceMethod)
    
    pub_bb.publish(markerArray)
    
    if paramVisualizeBoundingboxes == True:
        imgDim = np.array([cv_image.shape[0],cv_image.shape[0],cv_image.shape[1],cv_image.shape[1]])
        imgCrop = (imgDim*crop).astype(int)
        cv2.rectangle(cv_image,(imgCrop[2],imgCrop[0]),(imgCrop[3],imgCrop[1]),[0,0,0],3)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        pub_image_visualize.publish(image_message)
    
    
ts_image = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, det_sub], 10)
ts_image.registerCallback(callback_image)

# main
def main():

    rospy.spin()

if __name__ == '__main__':
    main()
