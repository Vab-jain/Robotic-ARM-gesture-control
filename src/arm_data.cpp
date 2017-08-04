#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "iostream"
#include <fstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_data");

	ros::NodeHandle node;

	tf::TransformBroadcaster br;
	
	// ros::Publisher arm_data = node.advertise<std_msgs::Float64MultiArray>("arm_data", 10);
	// ros::Publisher arm_data = node.advertise<arm::Data>("arm_data", 10);

	tf::StampedTransform transform_shoulder_prev, transform_shoulder_curr, transform_elbow_prev, transform_elbow_curr, transform_shoulder_diff, transform_elbow_diff;

	tf::TransformListener listener;

	ros::Rate rate(2.0);

	if(node.ok()){
		try{
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), transform_shoulder_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), transform_shoulder_prev);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), transform_elbow_curr);
			listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), transform_elbow_prev);
		}
	    catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
	         // ros::Duration(1.0).sleep();
	    }
	}

	while (node.ok()){

	    try{
	    	listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), transform_shoulder_curr);
	    	listener.lookupTransform("/openni_depth_frame", "/right_elbow_1", ros::Time(0), transform_elbow_curr);
	    }
	    catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
	         ros::Duration(1.0).sleep();
	    }


	    // cout << *transform_shoulder_curr.getRotation() << endl;
	    // cout << *transform_shoulder_prev.getRotation() << endl;

	    // const tf::Quaternion &quaternion_shoulder_diff = transform_shoulder_curr.getRotation() - transform_shoulder_prev.getRotation();
	    const tf::Quaternion &quaternion_shoulder_diff = transform_shoulder_curr.getRotation();
	    // ROS_ERROR(quaternion_shoulder_diff);
	    // const tf::Quaternion &quaternion_elbow_diff = transform_elbow_curr.getRotation() - transform_elbow_prev.getRotation();
	    const tf::Quaternion &quaternion_elbow_diff = transform_elbow_curr.getRotation();
	    // ROS_ERROR("END OF STUPID CODE");

	    double roll, pitch, yaw;


	    ofstream outfile;
	    outfile.open("/dev/ttyUSB2");

	    tf::Matrix3x3 m(quaternion_shoulder_diff);
	    m.getRPY(roll, pitch, yaw);

		outfile << roll << endl << pitch << endl << yaw <<endl;	    

	    m = tf::Matrix3x3 (quaternion_elbow_diff);
	    m.getRPY(roll,pitch,yaw);


	    outfile << roll << endl << pitch << endl << yaw <<endl;

   	    cout << *quaternion_shoulder_diff << endl;
	    cout << "roll" << endl;
	    cout << roll << endl;
	    cout << "pitch" << endl;
	    cout << pitch << endl;
	    cout << "yaw" << endl;
	    cout << yaw << endl;

	    outfile.close();

	    rate.sleep();


	    // std_msgs::Float64MultiArray array;

	    // arm::Data diff;
	    // diff.s_r = roll;
	    // diff.s_p = pitch;
	    // diff.s_y = yaw;

	    // diff.e_r = roll;
	    // diff.e_p = pitch;
	    // diff.e_y = yaw;
	    

	    // array.data.pushback(roll);
	    // array.data.pushback(pitch);
	    // array.data.pushback(yaw);


	    // array.data.pushback(roll);
	    // array.data.pushback(pitch);
	    // array.data.pushback(yaw);

	    // geometry_msgs::Quaternion shoulder_diff, elbow_diff;
	    // shoulder_diff = quaternion_shoulder_diff;
	    // shoulder_diff.w = quaternion_shoulder_diff.w;
	    // shoulder_diff.x = quaternion_shoulder_diff.x;
	    // shoulder_diff.y = quaternion_shoulder_diff.y;
	    // shoulder_diff.z = quaternion_shoulder_diff.z;
	    // elbow_diff = quaternion_elbow_diff;
	    // elbow_diff.w = quaternion_elbow_diff.w;
	    // elbow_diff.x = quaternion_elbow_diff.x;
	    // elbow_diff.y = quaternion_elbow_diff.y;
	    // elbow_diff.z = quaternion_elbow_diff.z;

	    // tf::quaternionTFToMsg(quaternion_shoulder_diff, shoulder_diff);
	    // tf::quaternionTFToMsg(quaternion_elbow_diff, elbow_diff);

	    // ROS_INFO("\nRoll : ");
	    // arm_data.publish(msg_roll);
	    // ROS_INFO("\nPitch : ");
	    // arm_data.publish(msg_pitch);
	    // ROS_INFO("\nYaw : ");
	    // arm_data.publish(msg_yaw);

	    // arm_data.publish(array);
	    // array.clear();

	    // arm_data.publish(quaternion_shoulder_diff);
	    // arm_data.publish(quaternion_elbow_diff);
	    // arm.data(diff);

	    // arm_data.publish(elbow_diff);

	}

	return 0;
}