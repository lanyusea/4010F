#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import sys
import cv2.cv as cv
from optparse import OptionParser
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from face.msg import facePoint

min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0


def processImage(img, depth_img):
    bridge=CvBridge()
    img = bridge.imgmsg_to_cv2(img, "bgr8")

    bridge2=CvBridge()
    depth_img= bridge2.imgmsg_to_cv2(depth_img, "passthrough")

    #cascade = cv.Load('/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
    cascade = cv.Load('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml')


    height, width, depth = img.shape
    gray = cv.CreateImage((width,height), 8, 1)
    small_img = cv.CreateImage((cv.Round(width / image_scale),
                   cv.Round (height / image_scale)), 8, 1)

    cv.CvtColor(cv.fromarray(img), gray, cv.CV_BGR2GRAY)

    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)

    cv.EqualizeHist(small_img, small_img)

    face_pub=rospy.Publisher('face_point',facePoint)
    if(cascade):
        t = cv.GetTickCount()
        faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        #print "detection time = %gms" % (t/(cv.GetTickFrequency()*1000.))
        if faces:
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the
                # bounding box of each face and convert it to two CvPoints
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(cv.fromarray(img), pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
                #publish it to face_point
                face_pub.publish((x+w/2)*image_scale,(y+h/2)*image_scale,depth_img[int(image_scale*(y+h/2)),int(image_scale*(x+w/2))])
    ##########################rosnode
    img_pub=rospy.Publisher('face_in_img',Image)
    bridge=CvBridge()
    msg=bridge.cv2_to_imgmsg(img, "bgr8")
    img_pub.publish(msg)
    #cv.ShowImage("result", img)



if __name__ == '__main__':

    rospy.init_node('image_listener',anonymous=True)
    image_sub = message_filters.Subscriber('/camera/rgb/image_raw',Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)

    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    ts.registerCallback(processImage)

    parser = OptionParser(usage = "usage: %prog [options] [filename|camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "../data/haarcascades/haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()

    #cascade = cv.Load('/home/pipikk/Downloads/opencv-2.4.8/data/haarcascades/haarcascade_frontalface_alt.xml')
    #cascade = cv.Load('/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
    cascade = cv.Load('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
