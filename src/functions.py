#!/usr/bin/env python

import rospy
import numpy as np
import cv2 
import os
import tf2_ros
import copy
from cv_bridge import CvBridge
import message_filters
#from image_geometry import PinholeCameraModel
import image_geometry

from sensor_msgs.msg import Image, CameraInfo
from boundingbox_msgs.msg import Boundingbox, Boundingboxes
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose

bridge = CvBridge()
cam_model = image_geometry.PinholeCameraModel()

    
def boundingboxTo3D(image, info, depth, bounding_boxes,pose,algorithmName,paramVisualizeBoundingboxes,paramEstDistanceMethod):
    
    cam_model.fromCameraInfo(info)    
    if paramVisualizeBoundingboxes == True:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    else:
        cv_image = []
    cv_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    
    # Construct Marker (single bounding box) and MarkerArray (all bounding boxes in an image)
    markerArray = MarkerArray()
    marker = Marker()
    marker.header = image.header
    #marker.header.frame_id = "Multisense/left_camera_frame"
    marker.lifetime = rospy.Duration(1.0) # One second
    marker.type = marker.CYLINDER
    marker.action = marker.DELETEALL
    marker.id = 0
    markerArray.markers.append(copy.deepcopy(marker))
    
    marker.action = marker.ADD
    
    
    
    
    # To visualize all bounding boxes in an image. They need unique ids. 
    # The id is incremented for each bounding boxes in an image.
    
    for idx, bounding_box in enumerate(bounding_boxes.boundingboxes): 
        ## Depth image        
        bbCropDepth = bbCoord_FromNormalized2Real(bounding_box,cv_depth.shape)
        
        # Crop out depth information from image. 
        depthCrop = cv_depth[bbCropDepth[2]:bbCropDepth[3],bbCropDepth[0]:bbCropDepth[1]];
        depthCropVec = depthCrop[:];
        
        # Get only valid depth values
        depthCropVec = depthCropVec[~np.isnan(depthCropVec)]
        
#        medianDepth = np.mean(depthCropVec)
#        print("EstimatedDistance(mean)",medianDepth)
#        medianDepth = np.median(depthCropVec)
#        print("EstimatedDistance(median)",medianDepth)
#        medianDepth = np.percentile(depthCropVec,10)
#        print("EstimatedDistance(10% percentile)",medianDepth)
        
        if paramEstDistanceMethod == 0:
            medianDepth = np.mean(depthCropVec)
        elif paramEstDistanceMethod == 1:
            medianDepth = np.median(depthCropVec)
        elif paramEstDistanceMethod == 2:
            medianDepth = np.percentile(depthCropVec,5)
        else:
            raise ValueError('Unknown method for estimate distance selected, set estDistanceMethod to either 0,1 or 2')
        
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

        
        marker.scale.x = bbWidth
        marker.scale.y = bbWidth
        marker.scale.z = bbHeight
        
        
        marker.pose.position.x = bbPosition[0]
        marker.pose.position.y = bbPosition[1]-bbHeight/2
        marker.pose.position.z = bbPosition[2]

        marker.pose.orientation = pose.orientation
        marker.id = idx
        marker.color.a = bounding_box.prob # Confidence
	
        if bounding_box.objectType == 0: # Human
            className = "human"
            colorRgb = (1.0, 0.0, 0.0)
        elif bounding_box.objectType == 1: # Other
            className = "other"
            colorRgb = (0.0, 1.0, 1.0)
        elif bounding_box.objectType == 2: # Unknown 
            className = "unknown"
            colorRgb = (0.0, 1.0, 1.0)
        elif bounding_box.objectType == 8: # Anomaly
            className = "anomaly"
            colorRgb = (1.0, 0.0, 1.0)
        else:
            className = "bad"
            colorRgb = (0.0, 0.0, 1.0)

        marker.ns = os.path.join(algorithmName, className)            
        marker.color.r = colorRgb[0]
        marker.color.g = colorRgb[1]
        marker.color.b = colorRgb[2]        

        ## RGB image
        if paramVisualizeBoundingboxes == True:
            bbCropRGB = bbCoord_FromNormalized2Real(bounding_box,cv_image.shape)
            cv2.rectangle(cv_image,(bbCropRGB[0],bbCropRGB[2]),(bbCropRGB[1],bbCropRGB[3]),(0,255,0),3)
            cv2.putText(cv_image,className + ', p' + str(round(marker.color.a,2)), (bbCropRGB[0],bbCropRGB[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)                
        #print "marker.pose.position: ", marker.pose.position
        # Make marker for each bounding box.
        if sum(np.isnan(np.array([marker.pose.position.x,marker.pose.position.y,marker.pose.position.z]))) == 0: 
            markerArray.markers.append(copy.deepcopy(marker))
            print "Valid markerArray"
    return markerArray,cv_image
        
def bbCoord_FromNormalized2Real(bounding_box,dimImage):
    bb = np.array([bounding_box.x*dimImage[1], bounding_box.y*dimImage[0], bounding_box.w*dimImage[1],bounding_box.h*dimImage[0]]).astype(int)
    bbCrop = np.array([bb[0],bb[0]+bb[2],bb[1],bb[1]+bb[3]]);
    bbCrop = np.maximum(bbCrop,0)
    bbCrop[0:2] = np.minimum(bbCrop[0:2],dimImage[1])
    bbCrop[2:4] = np.minimum(bbCrop[2:4],dimImage[0])
    # xmin,xmax,ymin,ymax
    # col_min,col_max,row_min,row_max
    return bbCrop;
