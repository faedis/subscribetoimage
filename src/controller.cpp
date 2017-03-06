/*
receives position of target and ptu
computes the ptu input and publishes that
receives waitkey and serves as interface for setting the PID, writing data etc.
*/


#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <sys/time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>

/*
//Specific input signal
std::vector<double> uprbs;
int prbscount = 0;
*/

class ControlAndMap{
	ros::NodeHandle nh_;
	ros::Subscriber pixelpos_sub_;
	ros::Subscriber waitkey_sub_;
	ros::Subscriber ptupos_sub_;
	ros::Publisher ptucmd_pub_;
	ros::Publisher targetpos_pub_;
private:
geometry_msgs::Pose2D ptucmd;
geometry_msgs::Pose2D targetpos;

std::ofstream myfile;
int filenumber = 0;
bool collectdata = false, writedata = false;
struct timeval t1, t2;
double elapsedTime;

bool errorarrived = false, ptuposarrived = false;
bool firstloop = true;
bool PTUctrl = false;
double alphaX = 0.079555; // correction  by measurements 1.055
double alphaY = 0.044814; // correction  by measurements 1.021
double pRes = 46.2857 / 3600 * M_PI / 180;
double tRes = 11.5714 / 3600 * M_PI / 180;
double hWidth = 320;
double hHeight = 180;
int uXmax= 10000, uXmin = 60;		// pos/second
int uYmax =10000, uYmin = 240;		// pos/second
double tolerance = 3;				// pixel		(other approach:0.00244; // rad)

std::vector<double> timeVec;
std::vector<double> exVec;
std::vector<double> eyVec;
std::vector<double> uxVec;
std::vector<double> uyVec;
std::vector<double> ixVec;
std::vector<double> iyVec;
std::vector<double> dxVec;
std::vector<double> dyVec;
std::vector<double> panposVec;
std::vector<double> tiltposVec;
std::vector<double> targetX;
std::vector<double> targetY;
int key = -1;
double 
pixelX = 0,				// local error pan in pixel
pixelY = 0,				// local error tilt in pixel
ptuX = 0,				// ptu pan rad
ptuY = 0,				// ptu tilt rad
yX = 0,					// ptu pan rad
yY = 0,					// ptu tilt rad
rX = 0,					// target pan rad
rY = 0,					// target tilt rad
eX = 0,					// local error pan rad
eY = 0,					// local error tilt rad
oeX = 0,				// old local error pan rad
oeY = 0,				// old local error tilt rad
ieX = 0,				// integral local error pan rad*s
ieY = 0,				// integral local error tilt rad*s
uXrad = 0,				// input pan in rad/second
uYrad = 0,				// input tilt in rad/second
uX = 0,					// input pan in ptu positions/second
uY = 0,					// input tilt in ptu positions/second
dt = 0.033,				// time step
Kp = 0, Ki = 0, Kd = 0;	// gains
public:
	ControlAndMap(){
		pixelpos_sub_ = nh_.subscribe("pixelpos",1,&ControlAndMap::pixelposCb,this);
		waitkey_sub_  = nh_.subscribe("waitkey",1,&ControlAndMap::waitkeyCb,this);
		ptupos_sub_ = nh_.subscribe("ptupos",1,&ControlAndMap::ptuposCb,this);
		ptucmd_pub_ = nh_.advertise<geometry_msgs::Pose2D>("ptucmd",1);
		targetpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("targetpos",1);
	}
	~ControlAndMap(){
	}
	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		pixelX = pixelpos->x;
		pixelY = pixelpos->y;
		// assign errors
		oeX = eX;
		oeY = eY;
		eX = -alphaX * (double)(pixelX - hWidth) / (double)hWidth;
		eY = -alphaY * (double)(pixelY - hHeight) / (double)hHeight;
		// update and publish target position if all information has arrvied
		errorarrived = true;
		if(ptuposarrived){ 
			rX = yX + eX;
			rY = yY + eY;
			errorarrived = false, ptuposarrived = false;
			targetpos.x = rX;
			targetpos.y = rY;
			targetpos.theta = 0;
			targetpos_pub_.publish(targetpos);
		}
		if(PTUctrl){
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
			if(firstloop){	// input without integrator and differentiator in rad
				uXrad = Kp*eX;
				uYrad = Kp*eY;
				firstloop = false;
			}
			else{			// full input in rad
				uXrad = Kp*eX + Ki*ieX + Kd*(eX-oeX)/dt;
				uYrad = Kp*eY + Ki*ieY + Kd*(eY-oeY)/dt;
			}
			ieX += eX*dt, ieY+= eY*dt; // update integral
			//////////////////////////////////
			//specific input signal
			/*
			if(prbscount<uprbs.size()){
				uXrad = uprbs[prbscount];
				uYrad = 0;
			}
			else{
				uXrad = 0;
				uYrad = 0;
			}
			prbscount++;
			*/
			//////////////////////////////////
			uX = uXrad/pRes;
			uY = uYrad/tRes;

			if(abs(uX)<uXmin){
				if(abs(pixelX-hWidth)>tolerance){//
					uX = copysign(uXmin,uX);
				}
				else uX = 0;
			}
			else if(abs(uX>uXmax)){
				uX = copysign(uXmax,uX);
			}
			if(abs(uY)<uYmin){
				if(abs(pixelY-hHeight)>tolerance){//
					uY = copysign(uYmin,uY);
				}
			else uY = 0;
			}
			else if(abs(uY>uYmax)){
				uY = copysign(uYmax,uY);
			}
			ptucmd.x = round(uX); //floor
			ptucmd.y = round(uY); //floor
			ptucmd.theta = 0;
			ptucmd_pub_.publish(ptucmd);
		}
		else{
			ptucmd.x = 0;
			ptucmd.y = 0;
			ptucmd.theta = 0;
			ptucmd_pub_.publish(ptucmd);
		}
	}
	void ptuposCb(const geometry_msgs::Pose2D::ConstPtr& ptupos){
		yX = ptupos->x * pRes;
		yY = ptupos->y * tRes;
		ptuposarrived = true;
		if(errorarrived){ 
			rX = yX + eX;
			rY = yY + eY;
			errorarrived = false, ptuposarrived = false;
			targetpos.x = rX;
			targetpos.y = rY;
			targetpos.theta = 0;
			targetpos_pub_.publish(targetpos);
		}
// store/write data:
		if(collectdata){
			timeVec.push_back(elapsedTime);
			exVec.push_back(eX);
			eyVec.push_back(eY);
			uxVec.push_back(uX*pRes);
			uyVec.push_back(uY*tRes);
			ixVec.push_back(ieX*Ki);
			iyVec.push_back(ieY*Ki);
			dxVec.push_back(Kd*(eX-oeX)/dt);
			dyVec.push_back(Kd*(eY-oeY)/dt);
			panposVec.push_back(yX);
			tiltposVec.push_back(yY);
			targetX.push_back(rX);
			targetY.push_back(rY);
		}
		if(writedata){
				writedata = false;
				std::stringstream ss;
				ss << "SmallTolKp" << Kp << "Ki" << Ki << "Kd" << Kd << "_" <<std::setfill('0')<<std::setw(3)<<
					filenumber <<".txt";
				std::string filename = ss.str();
				myfile.open(filename.c_str());
				ROS_INFO("file opened %s \n",filename.c_str());
				myfile << "Kp	Ki	Kd \n";
				myfile << Kp << "," << Ki << "," << Kd << "\n";
				myfile << "t, ex, ey, ux, uy, ix, iy, dx, dy, xy, yy, rx, ry\n";
				for (int i = 0; i < timeVec.size(); i++) {
					myfile << std::setprecision(14) << timeVec[i] << "," <<
						std::setprecision(8) << exVec[i] << "," <<
						std::setprecision(8) << eyVec[i] << "," <<
						std::setprecision(8) << uxVec[i] << "," <<
						std::setprecision(8) << uyVec[i] << "," <<
						std::setprecision(8) << ixVec[i] << "," <<
						std::setprecision(8) << iyVec[i] << "," <<
						std::setprecision(8) << dxVec[i] << "," <<
						std::setprecision(8) << dyVec[i] << "," <<
						std::setprecision(8) << panposVec[i] << "," <<
						std::setprecision(8) << tiltposVec[i] << "," <<
						std::setprecision(8) << targetX[i] << "," <<
						std::setprecision(8) << targetY[i] << "\n";
				}
				myfile.close();
				ROS_INFO("Data Written to File\n");
				filenumber++;
		}
	}
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		//ROS_INFO("waitkeyarrived\n");
		key = waitkey->data;
		switch(key){
			case -1:
				break;
			case 115: //s: start ptu control
				gettimeofday(&t1, NULL);
				PTUctrl ^= true;
				collectdata ^= true;
				if(collectdata){
					timeVec.clear();
					exVec.clear();
					eyVec.clear();
					uxVec.clear();
					uyVec.clear();
					ixVec.clear();
					iyVec.clear();
					dxVec.clear();
					dyVec.clear();
					panposVec.clear();
					tiltposVec.clear();
					targetX.clear();
					targetY.clear();
				}
				eX = 0, eY = 0, oeX = 0; oeY = 0, ieX=0, ieY=0;
				firstloop = true;
				if(PTUctrl) ROS_INFO("\n!!PTUctrl is ON!!\n");
				else ROS_INFO("\n!!PTUctrl is OFF!!\n");
				break;
			case 112: // p: set new gain
				ptucmd.x = 0;
				ptucmd.y = 0;
				ptucmd.theta = 0;			
				ptucmd_pub_.publish(ptucmd);
				firstloop = true;
				PTUctrl = false;
				ROS_INFO("\n!!PTUctrl is OFF!!\nKp: %f\nPlease enter new P gain: \n",Kp);
				std::cin >> Kp;
				ROS_INFO("New PID is: Kp %f Ki %f Kd %f\n",Kp,Ki,Kd);
				break;
			case 100: // d: set new diff gain
				ptucmd.x = 0;
				ptucmd.y = 0;
				ptucmd.theta = 0;			
				ptucmd_pub_.publish(ptucmd);
				firstloop = true;
				PTUctrl = false;
				ROS_INFO("\n!!PTUctrl is OFF!!\nKd: %f\nPlease enter new D gain: \n",Kd);
				std::cin >> Kd;
				ROS_INFO("New PID is: Kp %f Ki %f Kd %f\n",Kp,Ki,Kd);
				break;
			case 110: // i: set new int gain
				ptucmd.x = 0;
				ptucmd.y = 0;
				ptucmd.theta = 0;			
				ptucmd_pub_.publish(ptucmd);
				firstloop = true;
				PTUctrl = false;
				ROS_INFO("\n!!PTUctrl is OFF!!\nKi: %f\nPlease enter new I gain: \n",Ki);
				std::cin >> Ki;
				ROS_INFO("New PID is: Kp %f Ki %f Kd %f\n",Kp,Ki,Kd);
				break;
			case 119: // w: write data to file
				writedata = true;
				break;
			case 114: // r: rehome PTU
				ROS_INFO("Rehome PTU\n");
				//prbscount = 0;
				PTUctrl = false;
				firstloop = true;
				break;
			case 27:
				ROS_INFO("Shutdown\n");
				ros::shutdown();
				break;
		}
	}
};

int main(int argc, char **argv){
// load specific input signal
/*
	std::string line;
	std::string::size_type sz;     // alias of size_t
	std::ifstream prbsfile ("/home/guetgf/catkin_ws/src/subscribetoimage/src/u_panprbs.txt");
	if (prbsfile.is_open())
	{
	while ( std::getline (prbsfile,line) )
	{
	  uprbs.push_back(std::stod(line,&sz));
	}
	prbsfile.close();
	}
	else{
		std::cout << "Unable to open file";
		return 0;
	}
*/

	ros::init(argc,argv,"controller_node");
	ControlAndMap cm;
	ros::spin();
	return 0;
}

