/*
receives frame and displayes them with and without target rectangle
window without target rectangle is used to interrupt with the waitkey
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <string>

std::string winNameraw = "Preview Frame";
std::string winNamedet = "Preview Detection";


class ImageShow{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pixelpos_sub_;
	ros::Publisher waitkey_pub_;
private:
cv::Mat frame;
double tside;
std_msgs::Int8 waitkey;
public:
	ImageShow() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ImageShow::imageCb, this);
		pixelpos_sub_ = nh_.subscribe("pixelpos",1,
			&ImageShow::pixelposCb,this);
		waitkey_pub_ = nh_.advertise<std_msgs::Int8>("waitkey",1);

		cv::namedWindow(winNameraw,cv::WINDOW_KEEPRATIO);
		cv::namedWindow(winNamedet,cv::WINDOW_KEEPRATIO);
	}

	~ImageShow(){
		cv::destroyWindow(winNameraw);
		cv::destroyWindow(winNamedet);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		frame = cv_ptr->image;
		if(!cv_ptr->image.empty()){
			cv::imshow(winNameraw,cv_ptr->image);
			waitkey.data = cv::waitKey(1);
			waitkey_pub_.publish(waitkey);
			if(waitkey.data == 27){
				ROS_INFO("Shutdown\n");
				ros::Duration(0.06).sleep();
				ros::shutdown();
			}
		}
	}
	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		if(!frame.empty()){
			cv::rectangle(frame, cv::Rect(pixelpos->x - pixelpos->theta/2, pixelpos->y - 
				pixelpos->theta/2, pixelpos->theta,pixelpos->theta)&cv::Rect(0,0,640,360),
				cv::Scalar(0,0,255),2,8,0);
			cv::imshow(winNamedet,frame);
			cv::waitKey(1);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cvwindowctrl_node");
	ImageShow is;
	ros::spin();
	return 0;
}
