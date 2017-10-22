#!/usr/bin/python
# -*- coding: utf8 -*-
#
# Original code taken from the RGB-D benchmark: http://vision.in.tum.de/data/datasets/rgbd-dataset/tools
#
# Original author: Juergen Sturm

import argparse
import sys
import os

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file with depth images, 
    adds the corresponding PointCloud2 messages, and saves it again into a bag file. 
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file (default: 0.0)',default=0.00)
    parser.add_argument('--duration', help='only process N seconds of input bag file (default: off)')
    parser.add_argument('--compress', help='compress output bag file', action='store_true')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()

    import roslib; 
    import rospy
    import rosbag
    import sensor_msgs.msg
    import cv
    from cv_bridge import CvBridge, CvBridgeError
    import struct
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-points.bag"
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print "  starting from: %s seconds"%(args.start)
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)

    inbag = rosbag.Bag(args.inputbag,'r')
    if args.compress:
        param_compression = rosbag.bag.Compression.BZ2
    else:
        param_compression = rosbag.bag.Compression.NONE
        
    outbag = rosbag.Bag(args.outputbag, 'w', compression=param_compression)
    #######################################################33
    depth_camera_info = None

    depth_image = None

    cortex = None
    ###########################################################
    nan = float('nan')
    bridge = CvBridge()
    frame = 0 
    
    time_start = None
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic == "/camera/depth/camera_info":
            depth_camera_info = msg
            continue
        if topic == "/camera/depth/image_raw" and depth_camera_info:
            depth_image = msg
        ###########################################################
            # store messages
            outbag.write("/camera/depth/camera_info",depth_camera_info,t)
            outbag.write("/camera/depth/image_raw",depth_image,t)
    
            # generate depth point cloud
            cv_depth_image = bridge.imgmsg_to_cv2(depth_image, "passthrough")

            centerX = depth_camera_info.K[2]
            centerY = depth_camera_info.K[5]
            depthFocalLength = depth_camera_info.K[0]
            depth_points = sensor_msgs.msg.PointCloud2()
            depth_points.header = depth_image.header
            depth_points.width = depth_image.width
            depth_points.height  = depth_image.height
            depth_points.fields.append(sensor_msgs.msg.PointField(
                name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            depth_points.fields.append(sensor_msgs.msg.PointField(
                name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            depth_points.fields.append(sensor_msgs.msg.PointField(
                name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
            depth_points.point_step = 16 
            depth_points.row_step = depth_points.point_step * depth_points.width
            buffer = []
            for v in range(depth_image.height):
                for u in range(depth_image.width):
                    d = cv_depth_image[v,u]
                    ptx = (u - centerX) * d / depthFocalLength;
                    pty = (v - centerY) * d / depthFocalLength;
                    ptz = d;
                    buffer.append(struct.pack('ffff',ptx,pty,ptz,1.0))
            depth_points.data = "".join(buffer)
            outbag.write("/camera/depth/points", depth_points, t)
            # consume the images
            depth_image = None
            continue
        if topic not in ["/camera/depth/camera_info","/camera/depth/image_raw"]:
            # anything else: pass thru
            outbag.write(topic,msg,t)

outbag.close()
print
