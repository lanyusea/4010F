#include <iostream>
#include <ros/ros.h>
#include <face/facePoint.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
using namespace std;

class SubscribeAndPublish
{
    public:
        SubscribeAndPublish()
        {
            pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
            sub = n.subscribe("/face_point", 1, &SubscribeAndPublish::callback, this);
            id = 0;
        }

        void callback(const face::facePoint& msg)
        {
            ros::Rate rate(10);
            int u = msg.x;
            int v = msg.y;
            float lambda = msg.distance;

            geometry_msgs::PointStamped source;
            source.point.y = -(u-320.5)*(lambda)/554.25;
            source.point.z = -(v-240.5)*(lambda)/554.25;
            source.point.x = lambda;

            geometry_msgs::PointStamped destination;

            try{
                listener.lookupTransform("/odom","/camera_rgb_frame",ros::Time::now()-ros::Duration(0.1), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }


            visualization_msgs::Marker marker;
            marker.header.frame_id = "/odom";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position.y = source.point.x+transform.getOrigin().y();
            marker.pose.position.x = source.point.y+transform.getOrigin().x();
            marker.pose.position.z = source.point.z+transform.getOrigin().z();

            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.id = id++;
            marker.lifetime = ros::Duration();
            pub.publish(marker);
            rate.sleep();
        }

    private:
        //You should init everything out of callback function.
        //especially for transform, it must wait for a period until transform published by tf
        //or you will suffer frame_not_exist problem since if you init it every time in callback function
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        int id;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
