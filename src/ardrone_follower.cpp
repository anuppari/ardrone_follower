#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "aruco.h"
#include "cvdrawingutils.h"

using namespace aruco;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=0.129;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
bool fly = false;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;

void fh(const sensor_msgs::Joy::ConstPtr&);
void cvTackBarEvents(int,void*);
int findParam ( std::string,int, char* );
void imageCb(const sensor_msgs::ImageConstPtr&);

cv_bridge::CvImagePtr cv_ptr;

geometry_msgs::Twist cmd;
float gain = 0.25;
float maxx= 0, minx = 0, maxy = 0, miny = 0;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  cv::namedWindow(OPENCV_WINDOW);
    ros::NodeHandle cmd_n;
    ros::NodeHandle h_n;
    ros::Publisher cmd_pub = cmd_n.advertise<geometry_msgs::Twist>("cmd_vel",1);
ros::Subscriber h_pub = h_n.subscribe<sensor_msgs::Joy>("/joy",1,fh);
ros::Rate loop_rate(100);

 image_sub = it.subscribe("/mv_BF001066/image_raw", 1, imageCb);
 //image_sub = it.subscribe("/ardrone/image_raw", 1, imageCb);

while(ros::ok())
{

  //image_pub = it.advertise("/image_converter/output_video", 1);
		cmd_pub.publish(cmd);
ros::spinOnce();
loop_rate.sleep();
}
  // Output modified video stream
  //image_pub.publish(cv_ptr->toImageMsg());
  ros::spin();
  return 0;
}

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
    MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i=0;i<TheMarkers.size();i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
    //print other rectangles that contains no valid markers
    /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    }*/

//draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i=0;i<TheMarkers.size();i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",MDetector.getThresholdedImage());
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {


    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    try
    {
        //read first image to get the dimensions
        TheInputImage = cv_ptr->image.clone();

        //read camera parameters if passed
	//TheIntrinsicFile = "/home/ncr/ncr_ws/src/ROSOpenCV/out_camera_calibration.yml";
	//TheIntrinsicFile = "/home/ncr/ncr_ws/src/ardrone_follower/cal.yml";
	TheIntrinsicFile = "/home/ncr/.ros/camera_info/mv_BF001066.yaml";
        if (TheIntrinsicFile!="") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }
        cv::Size s1 = TheCameraParameters.CameraMatrix.size();
        cv::Size s2 = TheCameraParameters.Distorsion.size();
        cout << s1 << endl;
        cout << s2 << endl;
        cv::Mat camMat = cv::Mat::eye(3,3,CV_64FC1);
        cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64FC1);

        //Configure other parameters
        if (ThePyrDownLevel>0)
            MDetector.pyrDown(ThePyrDownLevel);


        //Create gui

        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
        MDetector.getThresholdParams( ThresParam1,ThresParam2);       
        MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        char key=0;
        int index=0;
Mat m44inv;
        //capture until press ESC or until the end of the video
        do 
        {
            TheInputImage = cv_ptr->image.clone();
            //copy image

            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of markers in the image passed
            cout << camMat << endl;
            cout << distCoeffs << endl;
            MDetector.detect(TheInputImage,TheMarkers,camMat,distCoeffs,TheMarkerSize);
            //chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
            AvrgTime.second++;
            cout<<"\rTime detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds nmarkers="<<TheMarkers.size()<< std::flush;
            //print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);
	    
            for (unsigned int i=0;i<TheMarkers.size();i++) {
                TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
        	Point cent(0,0);
        	for (int j=0;j<4;j++)
        	{
            		cent.x+=TheMarkers[i][j].x;
            		cent.y+=TheMarkers[i][j].y;
        	}
                cent.x/=4.;
                cent.y/=4.;
		circle(TheInputImageCopy, cent, 3, CV_RGB(0,255,0));
		cout << endl << "Center: " << cent << endl;
            }
	    cv::Mat m44=cv::Mat::eye(4,4,CV_32FC1);
            if (TheMarkers.size()!=0){

    		cv::Mat m33(3,3,CV_32FC1);
    		cv::Rodrigues(TheMarkers[0].Rvec, m33);
    		for (int i=0;i<3;i++){
        		for (int j=0;j<3;j++){
            			m44.at<float>(i,j)=m33.at<float>(i,j);
			}
		}
    		//now, add translation information
    		for (int i=0;i<3;i++){
        		m44.at<float>(i,3)=TheMarkers[0].Tvec.at<float>(0,i);
		}
    		//invert the matrix
		m44inv = m44.inv();
		cv::Mat cdmatrix = cv::Mat::zeros(3,3,CV_32FC1);
	gain = 0.1;
		float vxground = gain*(0 - m44inv.at<float>(0,3));
		float vyground = gain*(0 - m44inv.at<float>(1,3));
		float vzground = gain*(2 - m44inv.at<float>(2,3));

		cv::Mat vground = cv::Mat::zeros(3,1,CV_32FC1);
		vground.at<float>(0,0) = vxground;
		vground.at<float>(1,0) = vyground;
		vground.at<float>(2,0) = vzground;
		if(vxground > maxx) maxx = vxground;
		if(vxground < minx) minx = vxground;
		if(vyground > maxy) maxy = vyground;
		if(vyground < miny) miny = vyground;
//cout << maxx << endl << minx << endl << maxy << endl << miny << endl;           
cout << vground << endl;
		cdmatrix.at<float>(0,1) = -1;
		cdmatrix.at<float>(1,0) = -1;
		cdmatrix.at<float>(2,2) = -1;
		Mat vdrone = cdmatrix*m33*vground;
		cmd.linear.x = vdrone.at<float>(0,0);
		cmd.linear.y = vdrone.at<float>(1,0);
		cmd.linear.z = vdrone.at<float>(2,0);	
if(fly) 
{cout << "fly" << endl;
cmd.linear.z = 0.5;
}
else cmd.linear.z = 0.00;	
	    }
	    else{

		cmd.linear.x = 0;
		cmd.linear.y = 0;
		cmd.linear.z = 0;	
	    }

cout << endl << m44inv << endl;		
cout << endl << m44 << endl;



            //print other rectangles that contains no valid markers
       /**     for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
                aruco::Marker m( MDetector.getCandidates()[i],999);
                m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
            }*/



            //draw a 3d cube in each marker if there is 3d info
            if (  TheCameraParameters.isValid())
                for (unsigned int i=0;i<TheMarkers.size();i++) {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                }
            //DONE! Easy, right?
            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
            cv::imshow("thres",MDetector.getThresholdedImage());

            key=cv::waitKey(100);//wait for key to be pressed
        }while(key!=27 && TheVideoCapturer.grab());

    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    

}

int findParam ( std::string param,int argc, char *argv[] )
{
    for ( int i=0; i<argc; i++ )
        if ( string ( argv[i] ) ==param ) return i;

    return -1;

}
void fh(const sensor_msgs::Joy::ConstPtr& msgs)
{
	fly = msgs->buttons[0];
if(fly) 
{cout << "fly" << endl;
cmd.linear.z = 0.5;
}
else cmd.linear.z = 0.00;

}

