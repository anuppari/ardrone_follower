#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "aruco.h"
#include "cvdrawingutils.h"

using namespace aruco;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";


 int iLowH = 0;
int iHighH = 179;

int iLowS = 128; 
int iHighS = 255;

int iLowV = 107;

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=0.129;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
bool fly = false;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1 = 11,ThresParam2 = 10;
int iThresParam1,iThresParam2;
int waitTime=0;

void fh(const sensor_msgs::Joy::ConstPtr&);
int findParam ( std::string,int, char* );
void imageCb(const sensor_msgs::ImageConstPtr&);

cv_bridge::CvImagePtr cv_ptr;

geometry_msgs::Twist cmd;
float gain = 0.25;
float maxx= 0, minx = 0, maxy = 0, miny = 0;
float LastX = -1, LastY = -1;

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
ros::Rate loop_rate(10);
 namedWindow("Control", CV_WINDOW_AUTOSIZE);
 // image_sub = it.subscribe("/mv_BF001066/image_raw", 1, imageCb);
 image_sub = it.subscribe("/ardrone/image_raw", 1, imageCb);

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

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
  
   int iLastX = -1; 
int iLastY = -1;

int iHighV = 255;
 //Create trackbars in "Control" window
createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
createTrackbar("HighH", "Control", &iHighH, 179);

createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
createTrackbar("HighS", "Control", &iHighS, 255);

createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
createTrackbar("HighV", "Control", &iHighV, 255);
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
    do{
        //read first image to get the dimensions
        TheInputImage = cv_ptr->image.clone();

        //read camera parameters if passed
	//TheIntrinsicFile = "/home/ncr/ncr_ws/src/ROSOpenCV/out_camera_calibration.yml";
	TheIntrinsicFile = "/home/ncr/ncr_ws/src/ardrone_follower/cal.yml";
        if (TheIntrinsicFile!="") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }

        Mat imgHSV;

cvtColor(TheInputImage, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

Mat imgThresholded;

inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
      
      
//morphological closing (fill small holes in the foreground)
dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
//morphological opening (remove small objects from the foreground)
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 


 Moments oMoments = moments(imgThresholded);

double dM01 = oMoments.m01;
double dM10 = oMoments.m10;
double dArea = oMoments.m00;

// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
if (dArea > 100)
{
//calculate the position of the ball
int posX = dM10 / dArea;
int posY = dM01 / dArea;        
        

iLastX = posX;
iLastY = posY;
circle(TheInputImage,Point(posX,posY),3,Scalar(0,255,0),2,8,0);
}
if(iLastX != -1)
{
	    gain = 0.1;
        
        float vx = gain*(float(TheInputImage.size().height)/2 - float(iLastY))/(float(TheInputImage.size().height)/2);//vdrone.at<float>(0,0);
        float vy = gain*(float(TheInputImage.size().width)/2 - float(iLastX))/(float(TheInputImage.size().height)/2);//vdrone.at<float>(1,0);
        if(vx > 0.05) vx = 0.05;
        if(vy > 0.05) vy = 0.05;
        if(vx < 0.02) vx = 0;
        if(vy < 0.02) vy = 0;
		//cdmatrix.at<float>(0,1) = -1;
		//cdmatrix.at<float>(1,0) = -1;
		//cdmatrix.at<float>(2,2) = -1;
		//Mat vdrone = cdmatrix*m33*vground;
		cmd.linear.x = vx; /*((float(TheInputImage.size().height)/2 - float(iLastY))-(float(TheInputImage.size().height)/2 - float(LastY)))**/
		cmd.linear.y = vy ;/*-1*((float(TheInputImage.size().width)/2 - float(iLastX))-(float(TheInputImage.size().width)/2 - float(LastX)))**/
		cmd.linear.z = 0;//vdrone.at<float>(2,0);	
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
LastX = iLastX;
LastY = iLastY;
imshow("Thresholded Image", imgThresholded); //show the thresholded image
imshow("Original", TheInputImage); //show the original image

        }while(waitKey(30)!=27 && TheVideoCapturer.grab());

    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

    // Update GUI Window
    

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

