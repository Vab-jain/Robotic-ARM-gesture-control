#include <ros/ros.h>
#include <tf/transform_listner.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char**argv)
{
	/* 
	Must call ros::init always. 'init' needs to see argc & argv for some reaseon.
	The third argument to init() is the name of the node.
	*/
	ros::init(argc,argv,"arm_transformer");
	//if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};     ????????????

	/*
    NodeHandle is the main access point to communications with the ROS system.
    The first NodeHandle constructed will fully initialize this node, and the last
    NodeHandle destructed will close down the node.
    */
	ros::NodeHandle node;

	/*
	The tf package provides an implementation of a TransformListener to help make the task of receiving transforms easier.
	To use the TransformListener, we need to include the 'tf/transform_listener.h' header file.
	*/
	tf::TransformListener listener;

	/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
	// why 3 different publisher ?????????
	ros::Publisher arm_data_shoulder = node.advertise<geometry_msgs::Twist>("arm_data_shoulder", 10);
	ros::Publisher arm_data_elbow = node.advertise<geometry_msgs::Twist>("arm_data_elbow", 10);
	ros::Publisher arm_data_hand = node.advertise<geometry_msgs::Twist>("arm_data_hand", 10);

	/*
	frequency of loop in 'Hz'
	*/
	ros::Rate rate(10.0);

	while(node.ok())
	{
		/*
		StampedTransfrom is a data strucrure of 'tf'
		Child class of Transform. Additional => time stamp
		*/
		tf::StampedTransform transform_right_shoulder,transform_right_elbow,transform_right_hand;

		try{

			/*
				Query the listener for a specific transformation
				Parameters are:
				#1	We want the transform from this frame ...
				#2	... to this frame.
				#3	The time at which we want to transform. Providing ros::Time(0) will just get us the latest available transform.
				#4	The object in which we store the resulting transform.
			*/

      		listener.lookupTransform("/arm_base_frame", "/right_shoulder", ros::Time(0), transform_right_shoulder);
      		listener.lookupTransform("/arm_base_frame", "/right_elbow", ros::Time(0), transform_right_elbow);
      		listener.lookupTransform("/arm_base_frame", "/right_hand", ros::Time(0), transform_right_hand);
    	}
    	catch (tf::TransformException &ex) {
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
		    continue;
	    }

	    /*
		geometry_msgs provides messages for common geometric primitives such as points, vectors, and poses
	    */
	    //can also use StampedTwist if we require TimeStamp
		geometry_msgs::Twist shoulder_pose, elbow_pose, hand_pose;

		// linear data
		shoulder_pose.linear.x = transform_right_shoulder.getOrigin().x();
		shoulder_pose.linear.y = transform_right_shoulder.getOrigin().y();
		shoulder_pose.linear.z = transform_right_shoulder.getOrigin().z();

		elbow_pose.linear.x = transform_right_elbow.getOrigin().x();
		elbow_pose.linear.y = transform_right_elbow.getOrigin().y();
		elbow_pose.linear.z = transform_right_elbow.getOrigin().z();

		hand_pose.linear.x = transform_right_hand.getOrigin().x();
		hand_pose.linear.y = transform_right_hand.getOrigin().y();
		hand_pose.linear.z = transform_right_hand.getOrigin().z();

		// angular data
		shoulder_pose.angular.x = atan2(sqrt(pow(transform_right_shoulder.getOrigin().y(), 2) + pow(transform_right_shoulder.getOrigin().z(), 2)), x );
		shoulder_pose.angular.y = atan2(sqrt(pow(transform_right_shoulder.getOrigin().x(), 2) + pow(transform_right_shoulder.getOrigin().z(), 2)), y );
		shoulder_pose.angular.z = atan2(sqrt(pow(transform_right_shoulder.getOrigin().x(), 2) + pow(transform_right_shoulder.getOrigin().y(), 2)), z );

		elbow_pose.angular.x = atan2(sqrt(pow(transform_right_elbow.getOrigin().y(), 2) + pow(transform_right_elbow.getOrigin().z(), 2)), x );
		elbow_pose.angular.y = atan2(sqrt(pow(transform_right_elbow.getOrigin().x(), 2) + pow(transform_right_elbow.getOrigin().z(), 2)), y );
		elbow_pose.angular.z = atan2(sqrt(pow(transform_right_elbow.getOrigin().x(), 2) + pow(transform_right_elbow.getOrigin().y(), 2)), z );

		hand_pose.angular.x = atan2(sqrt(pow(transform_right_hand.getOrigin().y(), 2) + pow(transform_right_hand.getOrigin().z(), 2)), x );
		hand_pose.angular.y = atan2(sqrt(pow(transform_right_hand.getOrigin().x(), 2) + pow(transform_right_hand.getOrigin().z(), 2)), y );
		hand_pose.angular.z = atan2(sqrt(pow(transform_right_hand.getOrigin().x(), 2) + pow(transform_right_hand.getOrigin().y(), 2)), z );

		arm_data_shoulder.publish(shoulder_pose);
		arm_data_elbow.publish(elbow_pose);
		arm_data_hand.publish(hand_pose);

	    rate.sleep();
	}
	//ros::spin();     ???

	return 0;
};