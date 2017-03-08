#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int8.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>

/*
Receives frame and processes it
publishes target position and size
*/

struct timeval t1, t2;
double elapsedTime;

class ImageProc{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber waitkey_sub_;
	ros::Publisher pixelpos_pub_;
private:
int key = 0;
cv::Mat frameThr, frameHSV;
cv::Point target;
double tside;
geometry_msgs::Pose2D pixelpos;
public:
	ImageProc() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ImageProc::imageCb, this);
		waitkey_sub_ = nh_.subscribe("waitkey",1,&ImageProc::waitkeyCb,this);
		pixelpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pixelpos",1);

//		cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_KEEPRATIO);
	}

	~ImageProc(){
//		cv::destroyWindow(OPENCV_WINDOW);
	}
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		key = waitkey->data;

		if(key == 27){
			ROS_INFO("Shutdown\n");
			ros::shutdown();
		}
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		//ROS_INFO("callback:");
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::cvtColor(cv_ptr->image,frameHSV, cv::COLOR_BGR2HSV);
		cv::inRange(frameHSV,cv::Scalar(75,40,40), cv::Scalar(105,255,255), frameThr);
		//moprh opening
		cv::erode(frameThr, frameThr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(frameThr, frameThr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		//morph closing
		cv::dilate(frameThr, frameThr, cv::getStructuringElement(
			cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(frameThr, frameThr, cv::getStructuringElement(
			cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::Moments frameMom = cv::moments(frameThr/255); // m01/m00 = posY, m10/m00 = posX
		if(frameMom.m00 > 300){
			pixelpos.theta = std::sqrt(frameMom.m00);
			pixelpos.x = frameMom.m10 / frameMom.m00;
			pixelpos.y = frameMom.m01 / frameMom.m00;

		}
		else{
			pixelpos.x = 320;
			pixelpos.y = 180;
			pixelpos.theta = 0;
		}

		// Delay of 16ms!!!! artificial
		gettimeofday(&t1,NULL);
		elapsedTime = 0;
		while(elapsedTime<13){
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		} 



		pixelpos_pub_.publish(pixelpos);
		//ROS_INFO("publish:");
	}
};




int main(int argc, char **argv)
{

	ros::init(argc, argv, "capturesubscribe_node");
	//ROS_INFO("artificial delay!!!\n");
	ImageProc ip;
	ros::spin();
	return 0;
}

