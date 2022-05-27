#include <odom_estimator.h>

OdomEstimator::OdomEstimator(std::string nameInit) : initialized(false)
{
	name = nameInit;

	firstMocap = true;
	xHat = Eigen::Matrix<float,13,1>::Zero();
	P = Eigen::Matrix<float,13,13>::Zero();
	Q = Eigen::Matrix<float,12,12>::Zero();

	//position variance
	P.block(0,0,3,3) = 0.1*Eigen::Matrix3f::Identity();//covariance
	R.block(0,0,3,3) = 0.000001*Eigen::Matrix3f::Identity();//measurment covariance
	Q.block(0,0,3,3) = 0.0001*Eigen::Matrix3f::Identity();//process covariance

	//orientation variance
	P.block(3,3,4,4) = 0.1*Eigen::Matrix4f::Identity();//covariance
	R.block(3,3,4,4) = 0.000001*Eigen::Matrix4f::Identity();//measurment covariance
	Q.block(3,3,3,3) = 0.0001*Eigen::Matrix3f::Identity();//process covariance

	//linear velocity variance
	P.block(7,7,3,3) = 0.1*Eigen::Matrix3f::Identity();//covariance
	Q.block(6,6,3,3) = 0.0001*Eigen::Matrix3f::Identity();//process covariance

	//angular velocity variance
	P.block(10,10,3,3) = 0.1*Eigen::Matrix3f::Identity();//covariance
	Q.block(9,9,3,3) = 0.0001*Eigen::Matrix3f::Identity();//process covariance

	//process jacobian
	// F = Eigen::Matrix<float,13,13>::Identity();
	F = Eigen::Matrix<float,13,13>::Identity();

	//noise jacobian
	L = Eigen::Matrix<float,13,12>::Zero();

	//measruement jacobian
	H = Eigen::Matrix<float,7,13>::Zero();
	H.block(0,0,3,3) = Eigen::Matrix3f::Identity();
	H.block(3,3,4,4) = Eigen::Matrix4f::Identity();
	HT = H.transpose();

    geometry_msgs::PoseStampedConstPtr pose_msg;
    pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/rail/nwu/pose_stamped", nh);
    if (pose_msg != NULL) {
        ROS_INFO_STREAM("GOT MY FIRST MOCAP MESSAGE");
        Eigen::Vector3f p(pose_msg->pose.position.x,pose_msg->pose.position.y,pose_msg->pose.position.z);
        Eigen::Vector4f q(pose_msg->pose.orientation.w,pose_msg->pose.orientation.x,pose_msg->pose.orientation.y,pose_msg->pose.orientation.z);
        q /= q.norm();
        xHat.segment(0,3) = p;
		xHat.segment(3,4) = q;
    }

    arucoPoseSub = nh.subscribe("/aruco_single/pose",1,&OdomEstimator::ArucoposeCB,this);
    poseSub = nh.subscribe("/rail/nwu/pose_stamped",1,&OdomEstimator::MocapPoseCB,this);
    cameraInfoSub = nh.subscribe("/camera/color/camera_info",1,&OdomEstimator::cameraInfoCB,this);
    odomPub = nh.advertise<nav_msgs::Odometry>(name+"/odomEKF",1);
    velPub = nh.advertise<geometry_msgs::TwistStamped>(name+"/velEKF",1);

	// float qwwang = -M_PI/2.0;
	// qww = Eigen::Vector4f(cos(qwwang/2.0),-sin(qwwang/2.0),0.0,0.0);
	// qww /= qww.norm();
}

void OdomEstimator::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr &msg) {
    ros::Time t = msg->header.stamp;

    if(!initialized){
        tLast = t;
        initialized = true;
        return;
    }

    float dt = (t-tLast).toSec();
    F.block(0,7,3,3) = dt*Eigen::Matrix3f::Identity();
    F.block(3,3,4,4) = Eigen::Matrix4f::Identity() + 0.5*dt*partialqDotq(xHat.segment(10,3));
    F.block(3,10,4,3) = 0.5*dt*partialqDotw(xHat.segment(3,4));

    L.block(0,0,3,3) = dt*Eigen::Matrix3f::Identity();
    L.block(3,3,4,3) = 0.5*dt*partialqDotw(xHat.segment(3,4));
    L.block(7,6,3,3) = dt*Eigen::Matrix3f::Identity();
    L.block(10,9,3,3) = dt*Eigen::Matrix3f::Identity();

    //predict
    // std::cout << std::endl << "+++++++++++++++++" << std::endl;
    //std::cout << "\n dt " << dt <<std::endl;
    std::cout << "\n F \n" << F <<std::endl;
    std::cout << "\n L \n" << L <<std::endl;

    //std::cout << "\n xHat \n" << xHat <<std::endl;
    //std::cout << "\n xDot(xHat) \n" << xDot(xHat) <<std::endl;
    xHat += (xDot(xHat)*dt);
    // xHat.segment(3,4) /= xHat.segment(3,4).norm();
    // // P += ((F*P + P*F.transpose() + Q)*dt);
    P = (F*P*F.transpose() + L*Q*L.transpose());
    //
    // std::cout << "\n F \n" << F <<std::endl;
    std::cout << "\n P \n" << P <<std::endl;

    std::cout << "\n state cov \n" << F*P*F.transpose() <<std::endl;
    std::cout << "\n noise cov \n" << L*Q*L.transpose() <<std::endl;
    tLast = t;

    publish_state();
}

void OdomEstimator::MocapPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    Eigen::Vector3f p(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    Eigen::Vector4f q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
    q /= q.norm();

    z.segment(0,3) = p;
    z.segment(3,4) = q;

    if ((xHat.segment(3,4) + q).norm() < (xHat.segment(3,4) - q).norm())
    {
        q *= -1.0;
    }

}

void OdomEstimator::ArucoposeCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    if(abs(z(0,1)) <= 0.25) {
        std::cout<<"Occlusion Measurement"<<std::endl;
        return;
    }
	//ros::Time t = msg->header.stamp;
	ros::Time t = msg->header.stamp;
	std::string frame_id = msg->header.frame_id;



	// p = rotatevec(p,qww);
	// q = getqMat(qww)*q;
	// q /= q.norm();



	if (firstMocap)
	{
//		xHat.segment(0,3) = p;
//		xHat.segment(3,4) = q;
		tLast = t;
		firstMocap = false;
		return;
	}


	float dt = (t-tLast).toSec();
	tLast = t;

	std::cout << "dt " << dt << std::endl;

	Eigen::Matrix<float,7,7> argK = H*P*HT+R;
	// std::cout << "\n argK \n" << argK <<std::endl;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargK(argK, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix<float,7,7> argKI = svdargK.solve(Eigen::Matrix<float,7,7>::Identity());
	// std::cout << "\n argKI \n" << argKI <<std::endl;
	Eigen::Matrix<float,13,7> K = P*HT*argKI;
	// std::cout << "\n K \n" << K <<std::endl;

	// std::cout << "\n z \n" << z <<std::endl;
	// std::cout << "\n xHat \n" << xHat.segment(0,7) <<std::endl;
	// std::cout << "\n z-xHat \n" << (z-xHat.segment(0,7)) <<std::endl;
	// std::cout << "\n K*(z-xHat) \n" << (K*(z-xHat.segment(0,7))).segment(0,7) <<std::endl;

	// std::cout << std::endl << xHat.segment(0,7) << std::endl;
	// std::cout << std::endl << P << std::endl;

	//perform chi squared test to make sure measurement okay
	Eigen::Matrix<float,7,1> error = z-xHat.segment(0,7);
	float chiTest = error.transpose()*argKI*error;
	float chi2 = 6.63; //chi^2 for 99%

	std::cout << "chi test value " << chiTest << std::endl;
	//if okay then use
	// if (chiTest < chi2)
	if (true)
	{
		xHat += (K*error);
		// std::cout << std::endl << "----------------" << std::endl;
		// xHat.segment(3,4) /= xHat.segment(3,4).norm();
		P = (Eigen::Matrix<float,13,13>::Identity() - K*H)*P;
	}
}

void OdomEstimator::publish_state() {

    Eigen::Vector4f qHat = xHat.segment(3,4)/xHat.segment(3,4).norm();
    Eigen::Vector3f vBody = rotatevec(xHat.segment(7,3),getqInv(qHat));
    Eigen::Vector3f wBody = xHat.segment(10,3);
    Eigen::Vector3f wWorld = rotatevec(xHat.segment(10,3),qHat);

    //set stamp and frame id
    geometry_msgs::Pose poseMsg;
    geometry_msgs::Twist velMsg;
    geometry_msgs::TwistStamped velBodyMsg;
    nav_msgs::Odometry odomMsg;


    odomMsg.child_frame_id = name;
    odomMsg.header.frame_id = "world";
    odomMsg.header.stamp = tLast;
    velBodyMsg.header.stamp = tLast;

    //set data
    poseMsg.position.x = xHat(0); poseMsg.position.y = xHat(1); poseMsg.position.z = xHat(2);
    poseMsg.orientation.w = qHat(0); poseMsg.orientation.x = qHat(1); poseMsg.orientation.y = qHat(2); poseMsg.orientation.z = qHat(3);

    velMsg.linear.x = xHat(7); velMsg.linear.y = xHat(8); velMsg.linear.z = xHat(9);
    velMsg.angular.x = wWorld(0); velMsg.angular.y = wWorld(1); velMsg.angular.z = wWorld(2);

    velBodyMsg.twist.linear.x = vBody(0); velBodyMsg.twist.linear.y = vBody(1); velBodyMsg.twist.linear.z = vBody(2);
    velBodyMsg.twist.angular.x = wBody(0); velBodyMsg.twist.angular.y = wBody(1); velBodyMsg.twist.angular.z = wBody(2);

    odomMsg.pose.pose = poseMsg;
    odomMsg.twist.twist = velMsg;

    int poseind = 0;
    int twistind = 0;
    for (int ii = 0; ii < 13*13; ii++)
    {
        int row = ii/13;
        int col = ii%13;

        //std::cout << "\n ***** \n row " << row << " col " << col;

        //p-p components
        if(row < 3 && col < 3)
        {
            //std::cout << " p-p" << std::endl;
            odomMsg.pose.covariance.at(poseind) = P(row,col);
            poseind++;
        }
            //p-qv components
        else if(row < 3 && col > 3 && col < 7)
        {
            //std::cout << " p-qv" << std::endl;
            odomMsg.twist.covariance.at(poseind) = P(row,row);
            poseind++;
        }
            //qv-p components
        else if(row > 3 && row < 7 && col < 3)
        {
            //std::cout << " qv-p" << std::endl;
            odomMsg.pose.covariance.at(poseind) = P(row,col);
            poseind++;
        }
            //qv-qv components
        else if(row > 3 && row < 7 && col > 3 && col < 7)
        {
            //std::cout << " qv-qv" << std::endl;
            odomMsg.pose.covariance.at(poseind) = P(row,col);
            poseind++;
        }
            //v-v components
        else if(row >= 7 && row < 10 && col >= 7 && col < 10)
        {
            //std::cout << " v-v" << std::endl;
            odomMsg.twist.covariance.at(twistind) = P(row,col);
            twistind++;
        }
            //v-w components
        else if(row >= 7 && row < 10 && col >= 10)
        {
            //std::cout << " v-w" << std::endl;
            odomMsg.twist.covariance.at(twistind) = P(row,col);
            twistind++;
        }
            //w-v components
        else if(row >= 10 && col >= 7 && col < 10)
        {
            //std::cout << " w-v" << std::endl;
            odomMsg.twist.covariance.at(twistind) = P(row,col);
            twistind++;
        }
            //w-w components
        else if(row >= 10 && col >= 10)
        {
            //std::cout << " w-w" << std::endl;
            odomMsg.twist.covariance.at(twistind) = P(row,col);
            twistind++;
        }

        //std::cout << "\n ------ \n";
    }


    //publish
    velPub.publish(velBodyMsg);
    odomPub.publish(odomMsg);

}

Eigen::Matrix4f OdomEstimator::partialqDotq(Eigen::Vector3f w)
{
	Eigen::Matrix4f pqDotq = Eigen::Matrix4f::Zero();
	pqDotq(0,1) = -w(0);
	pqDotq(0,2) = -w(1);
	pqDotq(0,3) = -w(2);
	pqDotq(1,0) = w(0);
	pqDotq(1,2) = w(2);
	pqDotq(1,3) = -w(1);
	pqDotq(2,0) = w(1);
	pqDotq(2,1) = -w(2);
	pqDotq(2,3) = w(0);
	pqDotq(3,0) = w(2);
	pqDotq(3,1) = w(1);
	pqDotq(3,2) = -w(0);
	return pqDotq;
}

Eigen::Matrix<float,4,3> OdomEstimator::partialqDotw(Eigen::Vector4f q)
{
	return B(q);
}

Eigen::Matrix<float,13,1> OdomEstimator::xDot(Eigen::Matrix<float,13,1> x)
{
	Eigen::Matrix<float,13,1> xDot = Eigen::Matrix<float,13,1>::Zero();
	xDot.segment(0,3) = x.segment(7,3);
	xDot.segment(3,4) = 0.5*B(x.segment(3,4))*x.segment(10,3);
	return xDot;
}
