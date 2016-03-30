#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense> // Defines MatrixXd, Matrix3d, Vector3d
#include <Eigen/Geometry> // Defines Quaterniond

Eigen::Matrix<double,4,3> diffMat(const Eigen::Quaterniond);

// need a class in order publish in the callback
class EKF
{
    ros::NodeHandle nh;
    ros::Publisher outputPub;
    ros::Subscriber targetVelSub;
    ros::Subscriber camVelSub;
    ros::Subscriber featureSub;
    tf::TransformListener tfl;
    
    // parameters
    string markerID;
    double q;   // EKF process noise scale
    double r;   // EKF measurement noise scale
    
    //states
    bool statesInitialized;
    Eigen::Vector3d xhat;
    Eigen::Quaternion qhat;
    Eigen::Matrix<double,7,7> P;         // Estimated covariance !!!!!!!!!! FIX DIMENSIONS !!!!!!!!!!!
    double lastVelTime;
    double lastImageTime;
    Eigen::Vector3d vTt;   // target linear velocity w.r.t. ground, expressed in target coordinate system
    Eigen::Vector3d wGTt;   // target angular velocity w.r.t. ground, expressed in target coordinate system
    Eigen::Vector3d vCc;   // camera linear velocity w.r.t. ground, expressed in camera coordinate system
    Eigen::Vector3d wGCc;  // camera angular velocity w.r.t. ground, expressed in camera coordinate system
    Eigen::Matrix<double,7,7> Q;     // EKF process noise covariance !!!!!!!!!! FIX DIMENSIONS !!!!!!!!!!!
    Eigen::Matrix<double,7,7> R;     // EKF measurement noise covariance !!!!!!!!!! FIX DIMENSIONS !!!!!!!!!!!
    Eigen::Matrix<double,7,7> H;   // EKF measurement model !!!!!!!!!! FIX DIMENSIONS !!!!!!!!!!!
public:
    EKF()
    {
        // Get Parameters
        ros::NodeHandle nhp("~");
        nhp.param<string>("markerID",markerID,"100");
        nhp.param<double>("q",q,10); // process noise
        nhp.param<double>("r",r,0.001); // measurement noise
        
        statesInitialized = false;
        lastVelTime = ros::Time::now().toSec();
        lastImageTime = lastVelTime;
        
        // Initialize EKF matrices
        Q = q*Matrix3d::Identity();
        R = r*Matrix2d::Identity();
        
        // Output publishers
        outputPub = nh.advertise<geometry_msgs::PoseStamped>("output",10);
        
        targetVelSub = nh.subscribe(targetName+"/odom",1,&EKF::targetVelCBdeadReckoning,this);
        camVelSub = nh.subscribe("image/body_vel",1,&EKF::camVelCB,this);
        featureSub = nh.subscribe("markers",1,&EKF::featureCB,this);
    }
    
    // Gets target velocities from turtlebot odometry
    void targetVelCBdeadReckoning(const nav_msgs::OdometryConstPtr& odom)
    {
        vTt << odom->twist.twist.linear.x,odom->twist.twist.linear.y,odom->twist.twist.linear.z;
        wGTt << odom->twist.twist.angular.x,odom->twist.twist.angular.y,odom->twist.twist.angular.z;
    }
    
    // Gets camera velocities and run predictor
    void camVelCB(const geometry_msgs::TwistStampedConstPtr& twist)
    {
        // Time
        ros::Time timeStamp = twist->header.stamp;
        double timeNow = timeStamp.toSec();
        double delT = timeNow - lastVelTime;
        lastVelTime = timeNow;
        
        // Camera velocities, expressed in camera coordinate system
        vCc << twist->twist.linear.x,twist->twist.linear.y,twist->twist.linear.z;
        wGCc << twist->twist.angular.x,twist->twist.angular.y,twist->twist.angular.z;
        
        // EKF prediction step
        ekf_predictor(delT);
        
        // Publish output
        publishOutput(timeStamp);
    }
    
    // Calculate linearization of dynamics (F) for EKF
    Matrix3d calculate_F(Vector3d xhat_,Vector3d vCc_,Vector3d vTc_,Vector3d wGCc_)
    {
        double vc1 = vCc_(0);        double vc2 = vCc_(1);        double vc3 = vCc_(2);
        double vq1 = vTc_(0);        double vq2 = vTc_(1);        double vq3 = vTc_(2);
        double w1 = wGCc_(0);        double w2 = wGCc_(1);        double w3 = wGCc_(2);
        double x1 = xhat_(0);        double x2 = xhat_(1);        double x3 = xhat_(2);
        
        F << 0, w3, -1*w2,
            -w3, 0, w1,
            w2, -w1, 0;
        return F;
    }
    
    void ekf_predictor(double delT)
    {
        // Target velocities expressed in camera coordinates
        Vector3d vTc = qhat*vTt;
        
        // Camera angular velocity expressed in target coordinates
        Vector3d wGCt = qhat.inverse()*wGCc;
        
        // Predictor
        xhat += (vTc - vCc - wGCc.cross(xhat))*delT
        Eigen::Vector4d qTemp(qhat.w(),qhat.x(),qhat.y(),qhat.z());
        qTemp += 0.5*diffMat(targetOrient)*targetAngVel*delT;
        qhat = Eigen::Quaterniond(qTemp(0),qTemp(1),qTemp(2),qTemp(3));
        qhat.normalize();
        
        // Predict Covariance
        Matrix3d F = calculate_F(xhat,vCc,vTc,wGCc);
        MatrixXd Pdot = F*P+P*F.transpose() + Q;
        P += Pdot*delT;
    }
    
    // Callback for estimator
    void featureCB(const geometry_msgs::PoseStampedConstPtr& pose)
    {
        // Time
        ros::Time timeStamp = center->header.stamp;
        
        // Pose measurements
        Eigen::Matrix<double,7,1> y(pose->pose.position.x,pose->pose.position.y,pose->pose.position.z,
                                    pose->pose.orientation.w,pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z);
        
        // EKF update
        Matrix<double,7,7> H = Eigne::Matrix<double,7,7>::setIdentity();
        Matrix<double,7,7> K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
        Eigen::Matrix<double,7,1> yhat;
        Eigen::Vector4d qTemp(qhat.w(),qhat.x(),qhat.y(),qhat.z());
        yhat << xhat, qTemp;
        Eigen::Matrix<double,7,1> poseUpdate = K*(y-yhat);
        xhat += poseUpdate.head<3>();
        qTemp += poseUpdate.tail<4>();
        qhat = Eigen::Quaterniond(qTemp(0),qTemp(1),qTemp(2),qTemp(3));
        qhat.normalize();
        
        P = (Matrix3d::Identity()-K*H)*P;
        
        // Publish output
        publishOutput(timeStamp);
    }
    
    // Method for publishing pose
    void publishOutput(ros::Time timeStamp)
    {
        // Publish output
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = timeStamp;
        poseMsg.pose.position.x = xhat(0);
        poseMsg.pose.position.y = xhat(1);
        poseMsg.pose.position.z = xhat(2);
        poseMsg.pose.orientation.w = qhat.w();
        poseMsg.pose.orientation.x = qhat.x();
        poseMsg.pose.orientation.y = qhat.y();
        poseMsg.pose.orientation.z = qhat.z();
        outputPub.publish(poseMsg);
    }

};//End of class EKF

// Calculate differential matrix for relationship between quaternion derivative and angular velocity.
// qDot = 1/2*B*omega 
// See strapdown inertial book. If quaternion is orientation of frame 
// B w.r.t N in the sense that nP = q*bP*q', omega is ang. vel of frame B w.r.t. N,
// i.e. N_w_B, expressed in the B coordinate system
// q = [w,x,y,z]
Eigen::Matrix<double,4,3> diffMat(const Eigen::Quaterniond q)
{
    Eigen::Matrix<double,4,3> B;
    B << -q.x(), -q.y(), -q.z(), q.w(), -q.z(), q.y(), q.z(), q.w(), -q.x(), -q.y(), q.x(), q.w();
    return B;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    
    EKF obj;
    
    ros::spin();
    return 0;
}
