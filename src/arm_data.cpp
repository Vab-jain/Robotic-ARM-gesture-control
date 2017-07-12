#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "iostream"

using namespace std;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_data");

	ros::NodeHandle node;

	tf::TransformBroadcaster br;
	
	ros::Publisher arm_data = node.advertise<geometry_msgs::Quaternion>("arm_data", 10);

	tf::StampedTransform transform_shoulder_prev, transform_shoulder_curr, transform_elbow_prev, transform_elbow_curr, transform_shoulder_diff, transform_elbow_diff;

	tf::TransformListener listener;

	ros::Rate rate(2.0);

	if(node.ok()){
		try{
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_2", ros::Time(0), transform_shoulder_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_2", ros::Time(0), transform_shoulder_prev);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_2", ros::Time(0), transform_elbow_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_2", ros::Time(0), transform_elbow_prev);
		}
	    catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
	         // ros::Duration(1.0).sleep();
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


	    // cout << *transform_shoulder_curr.getRotation() << endl;
	    // cout << *transform_shoulder_prev.getRotation() << endl;

	    const tf::Quaternion &quaternion_shoulder_diff = transform_shoulder_curr.getRotation() - transform_shoulder_prev.getRotation();
	    // ROS_ERROR(quaternion_shoulder_diff);
	    const tf::Quaternion &quaternion_elbow_diff = transform_elbow_curr.getRotation() - transform_elbow_prev.getRotation();
	    ROS_ERROR("END OF STUPID CODE");
	    tf::Matrix3x3 m(quaternion_shoulder_diff);
	    double roll, pitch, yaw;

	    m.getRPY(roll, pitch, yaw);

	    cout << *quaternion_shoulder_diff << endl;
	    cout << "roll" << endl;
	    cout << roll << endl;
	    cout << "pitch" << endl;
	    cout << pitch << endl;
	    cout << "yaw" << endl;
	    cout << yaw << endl;

	    geometry_msgs::Quaternion shoulder_diff, elbow_diff;

	    tf::quaternionTFToMsg(quaternion_shoulder_diff, shoulder_diff);
	    tf::quaternionTFToMsg(quaternion_elbow_diff, elbow_diff);

	    arm_data.publish(shoulder_diff);
	    arm_data.publish(elbow_diff);

		rate.sleep();

	}

	return 0;
}