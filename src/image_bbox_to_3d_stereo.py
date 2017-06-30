#!/usr/bin/env python

import rospy
import tf2_ros
from cv_bridge import CvBridge
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from boundingbox_msgs.msg import Boundingboxes
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
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
topicBBoxIn = rospy.get_param(nodeName+'/topicBBoxIn', nodeName+'UnknownInputTopic')

targetFrame = rospy.get_param(nodeName+'/targetFrame', nodeName+'UnknownFrameId')
algorithmName = rospy.get_param(nodeName+'/algorithmName', nodeName+'NotDefined')

# Get subscripers.
image_sub = message_filters.Subscriber(topicCamImg, Image)
info_sub = message_filters.Subscriber(topicCamInfo, CameraInfo)
depth_sub = message_filters.Subscriber(topicDepth, Image)
bb_sub = message_filters.Subscriber(topicBBoxIn, Boundingboxes)

# Name of output topics from launch-file. 
topicBBoxOut = rospy.get_param(nodeName+'/topicBBoxOut', nodeName+'/BBox3d') 
topicVisualizeOut = rospy.get_param(nodeName+'/topicVisualizeOut', nodeName+'/ImageBBox3d')

# Publishers
pub_bb = rospy.Publisher(topicBBoxOut, MarkerArray , queue_size=0)
if paramVisualizeBoundingboxes == True:
	pub_image_visualize = rospy.Publisher(topicVisualizeOut, Image , queue_size=0)

topicParts = [strPart for strPart in topicBBoxIn.split('/') if strPart is not '']

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

bridge = CvBridge()

    
def callback_bb(image, info, depth, bounding_boxes):
    
    
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
        
        pose.orientation = trans.transform.rotation
        #print("pose.orientation:",pose.orientation)
    except Exception as e: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #print("Except",e.message,e.args)
        print("WARNING: In image_bbox_to_3d_stereo node. No transform was found. 2d detection is NOT converted to 3d detection. A markerArray is not published",e.message,e.args)
        pose.orientation.w = 1
        validTranform = False
    
    if validTranform: 
        markerArray, cv_image = boundingboxTo3D(image, info, depth, bounding_boxes,pose,algorithmName,paramVisualizeBoundingboxes,paramEstDistanceMethod)
        pub_bb.publish(markerArray)
    
    if paramVisualizeBoundingboxes:
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        pub_image_visualize.publish(image_message)


ts_bb = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, bb_sub], 10)
ts_bb.registerCallback(callback_bb)

# main
def main():

    rospy.spin()

if __name__ == '__main__':
    main()
