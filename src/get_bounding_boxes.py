#!/usr/bin/env python

import rospy
import time
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from collections import namedtuple
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt16
from msg_boundingbox.msg import Boundingbox, Boundingboxes

rospy.init_node('get_boundingbox_distance', anonymous=False)

bridge = CvBridge()

#def callback(image, camera_info):
#    print('Base')
    
def callback_bb(image, depth, bounding_boxes):
    cv_image = bridge.imgmsg_to_cv2(image, "mono8")
    cv_depth = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    for idx, bounding_box in enumerate(bounding_boxes.boundingboxes): 
        print(idx)
        print(bounding_box)
    print('Synchronized image and bounding boxes')


image_sub = message_filters.Subscriber('/Multisense/left/image_rect_color', Image)
depth_sub = message_filters.Subscriber('/Multisense/depth', Image)
bb_sub = message_filters.Subscriber('/yolo/BBox/Multisense', Boundingboxes)

info_sub = message_filters.Subscriber('/Multisense/left/image_rect_color/camera_info', CameraInfo)

#ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
#ts.registerCallback(callback)

ts_bb = message_filters.TimeSynchronizer([image_sub, depth_sub, bb_sub], 10)
ts_bb.registerCallback(callback_bb)


# main
def main():
    
#    for iTopic in range(0,len(topics)): 
#        strParts = topics[iTopic].split('/')
#        topicOutName = '/ism/' + strParts[2] + '/' + strParts[3] + '/' + strParts[4] + '/'
#        print 'image2ism    is subscriping to topic "', topics[iTopic], '" and publishing "', topicOutName, '"'
#        rospy.Subscriber(topics[iTopic], Image, callbackDetectionImageReceived, queue_size=1)
    
    #rospy.Timer(rospy.Duration(timeBetweenEvaluation), EvaluateHumanAwareness)
    rospy.spin()

if __name__ == '__main__':
    main()
    
#from collections import namedtuple
#from std_msgs.msg import Float64MultiArray
#from std_msgs.msg import UInt16
#from sensor_msgs.msg import Image
#from nav_msgs.msg import OccupancyGrid
#import sys
#sys.path.append("/usr/lib/python2.7/dist-packages")
#import cv2
#from cv_bridge import CvBridge, CvBridgeError
#import numpy as np
#from geometry_msgs.msg import Pose, Point, Quaternion
#from ismFunctions import inversePerspectiveMapping, image2ogm
#
#rospy.init_node('image2ism', anonymous=True)
#nodeName = rospy.get_name()
#print "NodeName:", nodeName
##objectTypeInt = rospy.get_param(nodeName+'/objectTypeInt', 1000) # 1000 is not specified. 0-19 is pascal classes. 20 is the pedestrian detector
#topicsInName = rospy.get_param(nodeName+'/topicIns', 'UnknownInputTopic') # 1000 is not specified. 0-19 is pascal classes. 20 is the pedestrian detector
#imageWidth = rospy.get_param(nodeName+'/imageWidth', 800) 
#imageHeight = rospy.get_param(nodeName+'/imageHeight', 600) 
#cam_xTranslation = rospy.get_param(nodeName+'/cam_xTranslation', 0) 
#cam_yTranslation = rospy.get_param(nodeName+'/cam_yTranslation', 0) 
#cam_zTranslation = rospy.get_param(nodeName+'/cam_zTranslation', 1.5)
#cam_pitch = rospy.get_param(nodeName+'/cam_pitch', 0.349)  #20*np.pi/180
#cam_yaw = rospy.get_param(nodeName+'/cam_yaw', 0.1745) # 10*pi/180
#cam_FOV = rospy.get_param(nodeName+'/cam_FOV', 0.349) # 20*pi/180
#grid_resolution = rospy.get_param(nodeName+'/grid_resolution', 0.05) # 10*pi/180
#grid_xSizeInM = rospy.get_param(nodeName+'/grid_xSizeInM', -1.0) # For values <0 length of X is scaled automatically
#grid_ySizeInM = rospy.get_param(nodeName+'/grid_ySizeInM', -1.0) # For values <0 length of Y is scaled automatically    
#    
#(Xvis, Yvis,rHorizon) = inversePerspectiveMapping(imageWidth, imageHeight, cam_xTranslation, cam_yTranslation, cam_zTranslation, cam_pitch, cam_yaw, cam_FOV);
#
#topics = topicsInName.split(' ')
#print topics
#pubImageObjs = list()
#pubImageObjsDictionary = dict()
#for iTopic in range(0,len(topics)): 
#    strParts = topics[iTopic].split('/')
#    topicOutName = '/ism/' + strParts[2] + '/' + strParts[3] + '/' + strParts[4] + '/'
#    # DICTIONARY IS USED: PUBLISHER IS RETURNED USING TOPIC INPUT NAME.
#    pubImageObjsDictionary[topics[iTopic]] = rospy.Publisher(topicOutName, OccupancyGrid, queue_size=1)
#    
#bridge = CvBridge()
#
#vectorLength = 6
#def callbackDetectionImageReceived(data):
#    cv_image = bridge.imgmsg_to_cv2(data, "mono8")
#    cv_image = cv2.resize(cv_image,(imageWidth, imageHeight))
#    #cv_image = cv2.imread('/home/repete/blank_ws/src/image_inverse_sensor_model/src/tmpImage.png',0)
#    # Hack
#    
#    strParts = data.header.frame_id.split('/')
#    objectType = strParts[-1]
#    if objectType == 'human':
#        objectExtent = 0.5
#    elif objectType == 'unknown':
#        objectExtent = 1.0
#    elif objectType == 'vehicle':
#        objectExtent = 2.0
##        if(strParts[-2] == 'semantic_segmentation'):
##            cv2.imwrite("/home/repete/Desktop/TmpImages/image" + str(data.header.seq) + ".png", cv_image);
#    elif objectType == 'water':
#        objectExtent = 0.0
#    elif objectType == 'grass':
#
#        objectExtent = 0.0
#    elif objectType == 'ground':
#        objectExtent = 0.0
#    elif objectType == 'shelterbelt':
#        objectExtent = 1.5
#    elif objectType == 'anomaly':
#        objectExtent = 0.5
#    elif objectType == 'heat':
#        objectExtent = 0.5   
#    else:
#        objectExtent = 0.0
#        
#    #(Xvis, Yvis) = inversePerspectiveMapping(cv_image.shape[1], cv_image.shape[0], rHorizon, 0, 0, 1.5, 20*np.pi/180, 10*np.pi/180, 20*np.pi/180);
#    #print objectType, objectExtent
#    #tStart = time.time()
#    grid, nGridX, nGridY, dist_x1, dist_y1,empty = image2ogm(Xvis,Yvis,cv_image,rHorizon,grid_xSizeInM,grid_ySizeInM,grid_resolution,objectExtent)
#    #print "Elapsed on Image2ism: ", time.time()-tStart
#    #image_message = bridge.cv2_to_imgmsg(grid, encoding="mono8")
#    #pubImage.publish(image_message)
#    
#    grid_msg = OccupancyGrid()
#    grid_msg.header.stamp = rospy.Time.now()
#    grid_msg.header.frame_id = "base_link_mount" 
#    grid_msg.info.resolution = grid_resolution
#    grid_msg.info.width = nGridX
#    grid_msg.info.height = nGridY
##    origin_x = dist_x1+cam_xTranslation;
##    origin_y = dist_y1+cam_yTranslation;
##    print dist_x1, dist_y1
#    origin_x = dist_x1
#    origin_y = dist_y1
#    origin_z = 0;    
#    grid_msg.info.origin = Pose(Point(origin_x, origin_y, origin_z),Quaternion(0, 0, 0, 1))
#    grid_msg.data = grid.flatten()
#    # print "Sequence value", data.header.frame_id
#    # HACK: LOADS THE OBJECT TYPE FROM THE FRAME ID:
#    #pubImageObjs[0].publish(grid_msg)
#    #print "data.header.frame_id:", data.header.frame_id, "objectType: ", objectType, "grid_msg.data.shape", grid_msg.data.shape, "pubImageObjsDictionary.keys()", pubImageObjsDictionary.keys()
#    pubImageObjsDictionary[data.header.frame_id].publish(grid_msg)
#
#
#
## main
#def main():
#    for iTopic in range(0,len(topics)): 
#        strParts = topics[iTopic].split('/')
#        topicOutName = '/ism/' + strParts[2] + '/' + strParts[3] + '/' + strParts[4] + '/'
#        print 'image2ism    is subscriping to topic "', topics[iTopic], '" and publishing "', topicOutName, '"'
#        rospy.Subscriber(topics[iTopic], Image, callbackDetectionImageReceived, queue_size=1)
#    
#    #rospy.Timer(rospy.Duration(timeBetweenEvaluation), EvaluateHumanAwareness)
#    rospy.spin()
#
#
#if __name__ == '__main__':
#    main()
