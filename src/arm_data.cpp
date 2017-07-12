#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_data");

	ros::NodeHandle node;

	tf::TransformBroadcaster br;
	
	ros::Publisher arm_data = node.advertise<geometry_msgs::Quaternion>("arm_data", 10);

	tf::StampedTransform transform_shoulder_prev, transform_shoulder_curr, transform_elbow_prev, transform_elbow_curr, transform_shoulder_diff, transform_elbow_diff;

	tf::TransformListener listener;

	ros::Rate rate(10.0);

	if(node.ok()){
		try{
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_2", ros::Time(0), transform_shoulder_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_2", ros::Time(0), transform_shoulder_prev);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_2", ros::Time(0), transform_elbow_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_2", ros::Time(0), transform_elbow_prev);
		}
	    catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
	         ros::Duration(1.0).sleep();
	    }
	}

	while (node.ok()){

	    try{
	    	listener.lookupTransform("/openni_depth_frame", "/right_shoulder_2", ros::Time(0), transform_shoulder_curr);
	    	listener.lookupTransform("/openni_depth_frame", "/right_elbow_2", ros::Time(0), transform_elbow_curr);
	    }
	    catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
	         ros::Duration(1.0).sleep();
	    }

	    tf::Quaternion quaternion_shoulder_diff(1,1,1,1);
	    tf::Quaternion quaternion_elbow_diff(1,1,1,1);

	    quaternion_shoulder_diff *= transform_shoulder_curr.getRotation() - transform_shoulder_prev.getRotation();
	    quaternion_elbow_diff *= transform_elbow_curr.getRotation() - transform_elbow_prev.getRotation();

	    geometry_msgs::Quaternion shoulder_diff, elbow_diff;

	    tf::quaternionTFToMsg(quaternion_shoulder_diff, shoulder_diff);
	    tf::quaternionTFToMsg(quaternion_elbow_diff, elbow_diff);

	    arm_data.publish(shoulder_diff);
	    arm_data.publish(elbow_diff);

		rate.sleep();

	}

	return 0;
}