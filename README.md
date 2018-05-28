# ROS-tips-tricks

Tips and tricks for Robot Operating System.
A few code snippets/functions that come in handy when developing with ROS.

------
### Single message
Receive only one message on topic (usefull for configuraitons):
```cpp
ros::NodeHandle nh;
sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", nh, 10.0);
```

------
### Get TF transform
Function that looks up the TF transform between parent and child frame and handles errors
```cpp
bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform) {
	std::string errMsg;
	tf::TransformListener tf_listener;
	if (!tf_listener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.6), ros::Duration(0.1), &errMsg)) {
		ROS_ERROR_STREAM("!!! transform | Unable to get pose from TF: " << errMsg);
		return false;
	} else {
		try {
			tf_listener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
		}
		catch (const tf::TransformException &e) {
			ROS_ERROR_STREAM("!!! | Error in lookupTransform of " << childFrame << " in " << refFrame);
			return false;
		}
	}
	return true;
}
```
------

### Quaternion to Yaw
How to get yaw from quaternion
```cpp
double q0 = msg->pose.orientation.w;
double q1 = msg->pose.orientation.x;
double q2 = msg->pose.orientation.y;
double q3 = msg->pose.orientation.z;
double yaw = atan2(2 * (q0 * q3 + q1 * q2),1 - 2 * (q2 * q2 + q3 * q3)) + 3.14159;   
```

------
### Publish Marker
Publish single marker sphere/arrow to topic

```cpp
ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("/markers", 10);
void publishMarker(double x, double y, int id, double color){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "laser";
	marker.header.stamp = ros::Time();
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 0.8;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = color;
	marker.lifetime = ros::Duration(0.5);
	pub_marker.publish(marker);
}

void publishMarkerArrow(double x, double y, double roll,double pitch, double yaw, int id){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "laser";
	marker.header.stamp = ros::Time();
	marker.id = id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.1;
	geometry_msgs::Quaternion q_msg = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
	marker.pose.orientation = q_msg;
	marker.scale.x = 1;
	marker.scale.y = 0.02;
	marker.scale.z = 0.02;
	marker.color.a = 0.8;
	marker.color.r = 0.0;
	marker.color.g = 0.9;
	marker.color.b = 0.9;
	marker.lifetime = ros::Duration(0.5);
	pub_marker.publish(marker);
}
```

------
### ROS CameraInfo to OpenCV matrices
Function that reads a message of type sensor_msgs::CameraInfo and saves camera matrix and distortion to matrices as are required by OpenCV functions.
```cpp
	cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;

	void camInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg){
      std::vector<double> dist = msg->D;
      cv::Mat distortionCoeffs_l = cv::Mat(1, msg->D.size(), cv::CV_64FC1, &dist[0]);

      double cam_array[9];
      std::copy(msg->K.begin(),msg->K.end(), std::begin(cam_array));
      cv::Mat cameraMatrix_l = cv::Mat(3,3, CV_64FC1, &cam_array);
      
      distCoeffs_l.copyTo(distCoeffs); //TODO: without that additional copy they become strange when function gets out of scope
      cameraMatrix_l.copyTo(cameraMatrix);

      ROS_INFO("Got calibration parameters");
      ROS_INFO_STREAM("Distortion coefs:" <<distortionCoeffs);
      ROS_INFO_STREAM("Camera matrix:"<<cameraMatrix);
}
```
