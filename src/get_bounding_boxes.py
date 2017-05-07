#!/usr/bin/env python

import rospy
import numpy as np
import cv2 
import os
from cv_bridge import CvBridge
import message_filters
#from image_geometry import PinholeCameraModel
import image_geometry

from sensor_msgs.msg import Image, CameraInfo
from msg_boundingbox.msg import Boundingbox, Boundingboxes
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

rospy.init_node('get_boundingbox_distance', anonymous=False)
nodeName = rospy.get_name()

# Get topic names from launch file.
topicCamImg = rospy.get_param(nodeName+'/topicCamImage', 'UnknownInputTopic') 
topicCamInfo = rospy.get_param(nodeName+'/topicCamInfo', 'UnknownInputTopic') 
topicDepth = rospy.get_param(nodeName+'/topicDepth', 'UnknownInputTopic') 
topicBBoxIn = rospy.get_param(nodeName+'/topicBBoxIn', 'UnknownInputTopic') 
topicBBoxOut = rospy.get_param(nodeName+'/topicBBoxOut', 'UnknownInputTopic') 


# Get subscripers.
image_sub = message_filters.Subscriber(topicCamImg, Image)
info_sub = message_filters.Subscriber(topicCamInfo, CameraInfo)
depth_sub = message_filters.Subscriber(topicDepth, Image)
bb_sub = message_filters.Subscriber(topicBBoxIn, Boundingboxes)

topicParts = [strPart for strPart in topicBBoxIn.split('/') if strPart is not '']

# Publishers
pub_bb = rospy.Publisher(topicBBoxOut, MarkerArray , queue_size=0)


bridge = CvBridge()
cam_model = image_geometry.PinholeCameraModel()

#def callback(image, camera_info):
#    print('Base')

#pub_bb = rospy.Publisher('/det/Multisense/bboxes', Marker , queue_size=0)

def bbCoord_FromNormalized2Real(bounding_box,dimImage):
    bb = np.array([bounding_box.x*dimImage[1], bounding_box.y*dimImage[0], bounding_box.w*dimImage[1],bounding_box.h*dimImage[0]]).astype(int)
    bbCrop = np.array([bb[0],bb[0]+bb[2],bb[1],bb[1]+bb[3]]);
    bbCrop = np.maximum(bbCrop,0)
    bbCrop[0:2] = np.minimum(bbCrop[0:2],dimImage[1])
    bbCrop[2:4] = np.minimum(bbCrop[2:4],dimImage[0])
    # xmin,xmax,ymin,ymax
    # col_min,col_max,row_min,row_max
    return bbCrop;
    
def callback_bb(image, info, depth, bounding_boxes):
    cam_model.fromCameraInfo(info)    
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    cv_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    
#    # ONLY FOR TESTING    
#    if len(bounding_boxes.boundingboxes) > 0: 
#        print("STORE DATA")
#        dirImage = "/home/pistol/Code/ros_workspaces/private/src/image_boundingbox_to_3d/testRGB.jpg";
#        dirDepth = '/home/pistol/Code/ros_workspaces/private/src/image_boundingbox_to_3d/testDepth.dat';
#        dirBoundingBox = '/home/pistol/Code/ros_workspaces/private/src/image_boundingbox_to_3d/testBoundingBox.dat';
#        cv2.imwrite(dirImage , cv_image );
#        with open(dirDepth, 'wb') as outfile:
#            p.dump(cv_depth, outfile, protocol=p.HIGHEST_PROTOCOL)
#        with open(dirBoundingBox, 'wb') as outfile:
#            p.dump(bounding_boxes, outfile, protocol=p.HIGHEST_PROTOCOL)
#    
#        cv_image = cv2.imread(dirImage)
#        cv_depth = p.load(open(dirDepth,"rb"))
#        bounding_boxes = p.load(open(dirBoundingBox,"rb"))
    markerArray = MarkerArray()
    marker = Marker()
    marker.header = image.header
    marker.lifetime = rospy.Duration(1) # One second
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    #marker.action = marker.DELETEALL
    
    bb_id = 0
    #dimImage = cv_depth.shape
    for idx, bounding_box in enumerate(bounding_boxes.boundingboxes): 
        ## Depth image        
        bbCropDepth = bbCoord_FromNormalized2Real(bounding_box,cv_depth.shape)
        
        
#        bbPoints = [np.array([bounding_box.x,bounding_box.y]),
#                    np.array([bounding_box.x,bounding_box.y+bounding_box.h]),
#                    np.array([bounding_box.x+bounding_box.w,bounding_box.y+bounding_box.h]),
#                    np.array([bounding_box.x+bounding_box.w,bounding_box.y])]
        
        # Crop out depth information from image. 
        depthCrop = cv_depth[bbCropDepth[2]:bbCropDepth[3],bbCropDepth[0]:bbCropDepth[1]];
        depthCropVec = depthCrop[:];
        
        # Get only valid depth values
        depthCropVec = depthCropVec[~np.isnan(depthCropVec)]
        
        medianDepth = np.median(depthCropVec)
        
        # tl: top-left corner, br: bottom-right corner
        xyPoint_tl = np.array([bbCropDepth[0],bbCropDepth[2]]);
        xyPoint_br = np.array([bbCropDepth[1],bbCropDepth[3]]);
        
        #for bbPoint in bbPoints:
        ray_tl = np.array(cam_model.projectPixelTo3dRay(xyPoint_tl))
        ray_br = np.array(cam_model.projectPixelTo3dRay(xyPoint_br))
        xyzPoint_tl = ray_tl*medianDepth
        xyzPoint_br = ray_br*medianDepth
        
        bbWidth = xyzPoint_br[0]-xyzPoint_tl[0]
        bbHeight = xyzPoint_br[1]-xyzPoint_tl[1] 
        bbPosition = np.array([xyzPoint_tl[0]+bbWidth/2,xyzPoint_br[1],(xyzPoint_tl[2]+xyzPoint_br[2])/2]) # x,y,z
        
#        print("bbCropDepth",bbCropDepth)
#        print("Mean",np.mean(depthCropVec))
#        print("Median",medianDepth)
#        print("ray_tl: ", ray_tl,"PointInSpace",xyzPoint_tl)
#        print("ray_br: ", ray_br,"PointInSpace",xyzPoint_br)
#        print("Width:",bbWidth,"Height",bbHeight,"Position_xyz",bbPosition)        
        
        ## RGB image
        bbCropRGB = bbCoord_FromNormalized2Real(bounding_box,cv_image.shape)
        cv2.rectangle(cv_image,(bbCropRGB[0],bbCropRGB[2]),(bbCropRGB[1],bbCropRGB[3]),(0,255,0),1)
        cv2.putText(cv_image,str(np.round(bbPosition,2)), (bbCropRGB[0],bbCropRGB[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)        

        marker.id = bb_id
        marker.scale.x = bbWidth
        marker.scale.y = bbWidth
        marker.scale.z = bbHeight
        
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = bbPosition[0]
        marker.pose.position.y = bbPosition[2]
        marker.pose.position.z = bbPosition[1]
        marker.color.a = bounding_box.prob # Confidence
        if bounding_box.objectType == 0: # Human
            marker.ns = os.path.join(topicParts[0], "human")
            colorRgb = [1.0, 0.0, 0.0]
        elif bounding_box.objectType == 1: # Other
            marker.ns = os.path.join(topicParts[0], "other")
            colorRgb = [0.0, 1.0, 1.0]
        
        elif bounding_box.objectType == 2: # Unknown (typically not to be dangered)
            marker.ns = os.path.join(topicParts[0], "unknown")
            colorRgb = [0.0, 1.0, 1.0]
        else:
            marker.ns = os.path.join(topicParts[0], "bad")
            colorRgb = [0.0, 0.0, 1.0]
        marker.color.r = colorRgb[0]
        marker.color.g = colorRgb[1]
        marker.color.b = colorRgb[2]        
        
        # Make marker for each bounding box.         
        markerArray.markers.append(marker)
        bb_id = bb_id+1
    pub_bb.publish(markerArray)
    
    #print("END: MARKER MARKER MARKER MARKER MARKER MARKER") 
    
    cv2.imshow('image',cv_image)
    cv2.waitKey(1)
    
        
        
    print('Synchronized image and bounding boxes')




#ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
#ts.registerCallback(callback)

ts_bb = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, bb_sub], 10)
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
