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

# Estimate distance type
paramEstDistanceMethod = rospy.get_param(nodeName+'/estDistanceMethod', 1) # 0: mean, 1: median, 2: 10%-percentile.

# Parameter to specify if bounding boxes are visualized in image. 
paramVisualizeBoundingboxes = rospy.get_param(nodeName+'/visualizeBoundingboxes', 'False') 


# Name of input topics from launch-file. 
topicCamImg = rospy.get_param(nodeName+'/topicCamImage', nodeName+'UnknownInputTopic') 
topicCamInfo = rospy.get_param(nodeName+'/topicCamInfo', nodeName+'UnknownInputTopic') 
topicDepth = rospy.get_param(nodeName+'/topicDepth', nodeName+'UnknownInputTopic') 
topicBBoxIn = rospy.get_param(nodeName+'/topicBBoxIn', nodeName+'UnknownInputTopic') 

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
pub_image_visualize = rospy.Publisher(topicVisualizeOut, Image , queue_size=0)

topicParts = [strPart for strPart in topicBBoxIn.split('/') if strPart is not '']



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
    if paramVisualizeBoundingboxes == True:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    cv_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    
    # Construct Marker (single bounding box) and MarkerArray (all bounding boxes in an image)
    markerArray = MarkerArray()
    marker = Marker()
    marker.header = image.header
    marker.lifetime = rospy.Duration(1) # One second
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    
    # To visualize all bounding boxes in an image. They need unique ids. 
    # The id is incremented for each bounding boxes in an image.
    bb_id = 0
    
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
            medianDepth = np.percentile(depthCropVec,10)
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

        marker.id = bb_id
        marker.scale.x = bbWidth
        marker.scale.y = bbWidth
        marker.scale.z = bbHeight
        
        marker.pose.orientation.w = 1.0
        #marker.pose.position.x = bbPosition[0]
        #marker.pose.position.y = bbPosition[2]
        #marker.pose.position.z = bbPosition[1]
        marker.pose.position.x = bbPosition[1]
        marker.pose.position.y = -bbPosition[0]
        marker.pose.position.z = -bbPosition[2]
        marker.color.a = bounding_box.prob # Confidence
        if bounding_box.objectType == 0: # Human
            marker.ns = os.path.join(topicParts[0], "human")
            colorRgb = (1.0, 0.0, 0.0)
        elif bounding_box.objectType == 1: # Other
            marker.ns = os.path.join(topicParts[0], "other")
            colorRgb = (0.0, 1.0, 1.0)
        
        elif bounding_box.objectType == 2: # Unknown (typically not to be dangered)
            marker.ns = os.path.join(topicParts[0], "unknown")
            colorRgb = (0.0, 1.0, 1.0)
        else:
            marker.ns = os.path.join(topicParts[0], "bad")
            colorRgb = (0.0, 0.0, 1.0)
        marker.color.r = colorRgb[0]
        marker.color.g = colorRgb[1]
        marker.color.b = colorRgb[2]        

        ## RGB image
        if paramVisualizeBoundingboxes == True:
            bbCropRGB = bbCoord_FromNormalized2Real(bounding_box,cv_image.shape)
            cv2.rectangle(cv_image,(bbCropRGB[0],bbCropRGB[2]),(bbCropRGB[1],bbCropRGB[3]),(0,255,0),1)
            cv2.putText(cv_image,str(np.round(bbPosition,2)), (bbCropRGB[0],bbCropRGB[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)                

        # Make marker for each bounding box.         
        markerArray.markers.append(marker)
        bb_id = bb_id+1

    pub_bb.publish(markerArray)
    
    if paramVisualizeBoundingboxes == True:
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        pub_image_visualize.publish(image_message)


ts_bb = message_filters.TimeSynchronizer([image_sub,info_sub, depth_sub, bb_sub], 10)
ts_bb.registerCallback(callback_bb)

# main
def main():
    rospy.spin()

if __name__ == '__main__':
    main()
