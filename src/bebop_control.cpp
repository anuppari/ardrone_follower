#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class bebop_control
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::Publisher takeoffPub;
    ros::Publisher landPub;
    ros::Publisher resetPub;
    ros::Publisher velCmdPub;
    ros::Subscriber joySub;
    ros::Subscriber wallSub;
    ros::Subscriber mocapVelSub;
    ros::Subscriber homogVelSub;
    ros::Subscriber bodyVelSub;
    tf::TransformListener tfl;
    
    // Parameters
    tf::Vector3 boundaryOffsetBottomLeft;
    tf::Vector3 boundaryOffsetTopRight;
    tf::Matrix3x3 kpLin;
    tf::Matrix3x3 kdLin;
    tf::Matrix3x3 kffLin;
    double kpAng;
    double kdAng;
    double kffAng;
    
    // States
    bool wallOverride;
    bool useHomog;
    bool useMocap;
    double lastVelTime;
    tf::Vector3 desLinVel;
    tf::Vector3 desAngVel;
    tf::Vector3 lastLinError;
    tf::Vector3 lastAngError;
    
public:
    bebop_control()
    {
        // Publishers
        velCmdPub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
        takeoffPub = nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
        landPub = nh.advertise<std_msgs::Empty>("bebop/land",1);
        resetPub = nh.advertise<std_msgs::Empty>("bebop/reset",1);
        
        // Initialize parameters
        boundaryOffsetBottomLeft = tf::Vector3(-1.75, -1.25, 0);
        boundaryOffsetTopRight = tf::Vector3(1.25, 1.25, 2.5);
        kpAng = 1;
        kdAng = 0.005;
        kffAng = 0.581;
        kpLin = tf::Matrix3x3(1.75, 0, 0,
                              0, 1.75, 0,
                              0, 0,    1);
        kdLin = tf::Matrix3x3(0.01, 0, 0,
                              0, 0.01, 0,
                              0, 0, 0.01);
        kdLin = tf::Matrix3x3(0.0573, 0, 0,
                              0, 0.073, 0,
                              0, 0, 0.98);
        
        // Initialize states
        wallOverride = false;
        useHomog = false;
        useMocap = false;
        lastVelTime = ros::Time::now().toSec();
        desLinVel = tf::Vector3(0,0,0);
        desAngVel = tf::Vector3(0,0,0);
        lastLinError = tf::Vector3(0,0,0);
        lastAngError = tf::Vector3(0,0,0);
            
        // Subscribers
        joySub = nh.subscribe("joy",1,&bebop_control::joyCB,this);
        wallSub = nh.subscribe("bebop/pose",1,&bebop_control::wallCB,this);
        bodyVelSub = nh.subscribe("bebop/body_vel",1,&bebop_control::bodyVelCB,this);
        mocapVelSub = nh.subscribe("bebop/body_vel",1,&bebop_control::mocapVelCB,this);
        homogVelSub = nh.subscribe("bebop/body_vel",1,&bebop_control::homogVelCB,this);
    }
    
    void wallCB(const geometry_msgs::Pose& poseMsg)
    {
        // Center
        tf::Vector3 center(0,0,0);
        try
        {
            tf::StampedTransform tf1;
            tf::StampedTransform tf2;
            tf::StampedTransform tf3;
            tf::StampedTransform tf4;
            tfl.lookupTransform("world","ugv1",ros::Time(0),tf1);
            tfl.lookupTransform("world","ugv2",ros::Time(0),tf2);
            tfl.lookupTransform("world","ugv3",ros::Time(0),tf3);
            tfl.lookupTransform("world","ugv4",ros::Time(0),tf4);
            center = (tf1.getOrigin() + tf2.getOrigin() + tf3.getOrigin() + tf4.getOrigin())/4;
        }
        catch(tf::TransformException ex)
        {
        }
        
        // Boundary
        tf::Vector3 boundaryBottomLeft = center + boundaryOffsetBottomLeft;
        tf::Vector3 boundaryTopRight = center + boundaryOffsetTopRight;
        
        // Exceeding wall
        bool leftWall = poseMsg.position.x <= boundaryBottomLeft.getX();
        bool rightWall = poseMsg.position.x >= boundaryTopRight.getX();
        bool frontWall = poseMsg.position.y <= boundaryBottomLeft.getY();
        bool backWall = poseMsg.position.y >= boundaryTopRight.getY();
        bool bottomWall = poseMsg.position.z <= boundaryBottomLeft.getZ();
        bool topWall = poseMsg.position.z >= boundaryTopRight.getZ();
        
        if (leftWall or rightWall or frontWall or backWall or bottomWall or topWall)
        {
            wallOverride = true;
            
            // velocity command
            tf::Vector3 velCmd(leftWall - rightWall, frontWall - backWall, bottomWall - topWall);
            tf::Vector3 velCmdBody = tf::quatRotate(tf::Quaternion(poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z, poseMsg.orientation.w),velCmd);
            
            // construct message and publish
            geometry_msgs::Twist twistMsg;
            twistMsg.linear.x = velCmdBody.getX();
            twistMsg.linear.y = velCmdBody.getY();
            twistMsg.linear.z = velCmdBody.getZ();
            velCmdPub.publish(twistMsg);
        }
        else
        {
            wallOverride = false;
        }
    }
    
    void bodyVelCB(const geometry_msgs::TwistStampedConstPtr& twist)
    {
        // Measurements
        tf::Vector3 actualLinVel(twist->twist.linear.x,twist->twist.linear.y,twist->twist.linear.z);
        tf::Vector3 actualAngVel(twist->twist.angular.x,twist->twist.angular.y,twist->twist.angular.z);
        
        // Errors
        double delT = twist->header.stamp.toSec() - lastVelTime;
        lastVelTime = twist->header.stamp.toSec();
        tf::Vector3 linError = desLinVel - actualLinVel;
        tf::Vector3 angError = desAngVel - actualAngVel;
        tf::Vector3 linErrorDot = (linError - lastLinError)/delT;
        tf::Vector3 angErrorDot = (angError - lastAngError)/delT;
        lastLinError = linError;
        lastAngError = angError;
        
        // Command
        tf::Vector3 linCMD = kpLin*linError + kdLin*linErrorDot + kffLin*actualLinVel;
        tf::Vector3 angCMD = kpAng*angError + kdAng*angErrorDot + kffAng*actualAngVel;
        
        // Construct message and publish
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = linCMD.getX();
        twistMsg.linear.y = linCMD.getY();
        twistMsg.linear.z = linCMD.getZ();
        twistMsg.angular.x = angCMD.getX();
        twistMsg.angular.y = angCMD.getY();
        twistMsg.angular.z = angCMD.getZ();
        velCmdPub.publish(twistMsg);
    }
    
    void joyCB(const sensor_msgs::JoyConstPtr& joyMsg)
    {
        useHomog = false;
        useMocap = false;
        if (joyMsg->buttons[1]) // b - reset
        {
            resetPub.publish(std_msgs::Empty());
        }
        else if (joyMsg->buttons[1]) // a - land
        {
            landPub.publish(std_msgs::Empty());
        }
        else if (joyMsg->buttons[1]) // y - takeoff
        {
            takeoffPub.publish(std_msgs::Empty());
        }
        else if (!wallOverride)
        {
            if (joyMsg->buttons[5]) // RB - use homog
            {
                useHomog = true;
            }
            else if (joyMsg->buttons[4]) // LB - use mocap
            {
                useMocap = true;
            }
            else
            {
                desLinVel = tf::Vector3(-1*joy_deadband(joyMsg->axes[0]), joy_deadband(-1*joyMsg->axes[1]), joy_deadband(joyMsg->axes[7]));
                desAngVel = tf::Vector3(-1*joy_deadband(joyMsg->axes[4]), joy_deadband(joyMsg->axes[3]), joy_deadband(joyMsg->axes[2]-joyMsg->axes[5]));
            }
        }
    }
    
    void homogVelCB(const geometry_msgs::TwistConstPtr& twist)
    {
        if (useHomog)
        {
            desLinVel = tf::Vector3(twist->linear.x,twist->linear.y,twist->linear.z);
            desAngVel = tf::Vector3(twist->angular.x,twist->angular.y,twist->angular.z);
        }
    }
    
    void mocapVelCB(const geometry_msgs::TwistConstPtr& twist)
    {
        if (useMocap)
        {
            desLinVel = tf::Vector3(twist->linear.x,twist->linear.y,twist->linear.z);
            desAngVel = tf::Vector3(twist->angular.x,twist->angular.y,twist->angular.z);
        }
    }
    
    double joy_deadband(double input_value)
    {
        double filtered_value = 0;
        if (std::abs(input_value) > 0.15)
        {
            filtered_value = input_value;
        }
        return filtered_value;
    }
    
}; // end bebop_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing_node");
    
    bebop_control obj;
    
    ros::spin();
    return 0;
}
