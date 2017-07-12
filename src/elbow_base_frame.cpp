#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

	/*
		To create a frame which is similar to camera frame but at right elbow position called "elbow_base_frame"
	*/

int main(int argc, char **argv)
{
	/* 
	Must call ros::init always. 'init' needs to see argc & argv for some reaseon.
	The third argument to init() is the name of the node.
	*/
	ros::init(argc, argv, "elbow_base_frame");
	
	/*
    NodeHandle is the main access point to communications with the ROS system.
    The first NodeHandle constructed will fully initialize this node, and the last
    NodeHandle destructed will close down the node.
    */
    ros::NodeHandle node;

    /*
	The tf package provides an implementation of a TransformBroadcaster to help make the task of publishing transforms easier.
	To use the TransformBroadcaster, we need to include the tf/transform_broadcaster.h header file.
    */
	tf::TransformBroadcaster br;
  	
  	/*
	Transfrom is a data strucrure of 'tf'
  	*/
  // transform elbow is the new frame we are creating.
  // transform_origin is to feed origin of transform_elbow frame
  // transform_rotation is to feed rotation of transform_elbow frame
  	tf::StampedTransform transform_elbow, transform_origin, transform_rotation;

  	tf::TransformListener listener;

  	//ros::Publisher right_shoulder_joint = node.advertise<geometry_msgs::Point>("right_shoulder_joint", 10);
  	
  	ros::Rate rate(10.0);
  	
  	while (node.ok()){
  	try{

      listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), transform_origin);

    	/*???????????????????		START 		??????????????????????*/
    	// listener.lookupTransform("/openni_depth_frame", "/openni_depth_frame", ros::Time(0), transform_rotation);
    	/*???????????????????		END			???????????????*/
    }
    catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
    }
    transform_elbow.setOrigin( transform_origin.getOrigin() );		//	origin is equals to right shoulder origin
    // transform_elbow.setRotation( );	// 	rotation is equals of openni_depth_frame
    
	/*
	Sending a transform with a TransformBroadcaster
	Requires four arguments:
	#1	First, we pass in the transform itself.
	#2	Now we need to give the transform being published a timestamp, we'll just stamp it with the current time, ros::Time::now()
	#3	Then, we need to pass the name of the parent frame of the link we're creating
	#4	Finally, we need to pass the name of the child frame of the link we're creating
	*/
	br.sendTransform(tf::StampedTransform(transform_elbow, ros::Time::now(), "openni_depth_frame", "elbow_base_frame"));

    rate.sleep();
  }

	return 0;
}