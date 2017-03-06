#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <sys/time.h>
#include <string>

/*
publishes that frame is arrived
publishes frame
*/

struct timeval t1, t2;
double elapsedTime = 0;
int key = 0;
void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
	key = waitkey->data;
}

int main(int argc, char** argv)
{
	// Ros
	ros::init(argc, argv, "capturepublish_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
	ros::Publisher grabbed_pub = nh.advertise<std_msgs::Bool>("grabbed",1);
	ros::Subscriber waitkey_sub_  = nh.subscribe("waitkey",1,waitkeyCb);
	//ros::Publisher waitkey_pub = nh.advertise<std_msgs::Int8>("waitkey",1);
	//ros::Subscriber pixelpos_sub = nh.subscribe("pixelpos",1,pixelposCb);
	sensor_msgs::ImagePtr msg;
	std_msgs::Bool grabbed;
	//std_msgs::Int8 waitkey;
	//geometry_msgs::Pose2D pixelpos;

	// OpenCV
	cv::Mat frame;
  	cv::VideoCapture cap;
	cap.open(0);
	if(!cap.isOpened()){
		std::cout << "Cannot open camera!!!\n";
		return -1;
	}
//	std::string winName = "Preview";
//	cv::namedWindow(winName, cv::WINDOW_KEEPRATIO);
	int fps = 30;
	int frHeight = 360;
	int frWidth = 640;
	cap.set(cv::CAP_PROP_FPS,fps);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT,frHeight);
	cap.set(cv::CAP_PROP_FRAME_WIDTH,frWidth);

	// Loop
	while (nh.ok()) {
		//gettimeofday(&t1, NULL);
		cap.grab();
		grabbed.data = true;
		grabbed_pub.publish(grabbed);
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//ROS_INFO("time grab: %f", elapsedTime);
		cap.retrieve(frame);
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//ROS_INFO("time retrieved: %f", elapsedTime);
		if(!frame.empty()){
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			pub.publish(msg);
		}
		ros::spinOnce();
		if(key == 27) {
			ROS_INFO("Shutdown\n");
			break;
		}	
	}
}
