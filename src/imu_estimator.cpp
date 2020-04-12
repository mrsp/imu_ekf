/*
 * imu_ekf 
 *
 * Copyright 2016-2017 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <iostream>
#include <algorithm>
#include <imu_ekf/imu_estimator.h>


void imu_estimator::loadparams() {

	ros::NodeHandle n_p("~");

	n_p.getParam("imu_topic_freq",freq);
	n_p.getParam("useVO",useVO);
	n_p.getParam("useOdom",useOdom);
	n_p.getParam("useTwist",useTwist);
	
	n_p.getParam("camera_vo_topic", camera_vo_topic);
	n_p.getParam("odom_topic", odom_topic);
	n_p.getParam("imu_topic", imu_topic);
	n_p.getParam("twist_topic",twist_topic);
}



void imu_estimator::loadIMUEKFparams()
{
	ros::NodeHandle n_p("~");
	n_p.getParam("biasAX", biasAX);
	n_p.getParam("biasAY", biasAY);
	n_p.getParam("biasAZ", biasAZ);
	n_p.getParam("biasGX", biasGX);
	n_p.getParam("biasGY", biasGY);
	n_p.getParam("biasGZ", biasGZ);

	n_p.getParam("AccSTDx", imuEKF->acc_qx);
	n_p.getParam("AccSTDy", imuEKF->acc_qy);
	n_p.getParam("AccSTDz", imuEKF->acc_qz);

	n_p.getParam("GyroSTDx", imuEKF->gyr_qx);
	n_p.getParam("GyroSTDy", imuEKF->gyr_qy);
	n_p.getParam("GyroSTDz", imuEKF->gyr_qz);

	n_p.getParam("AccBiasSTDx", imuEKF->accb_qx);
	n_p.getParam("AccBiasSTDy", imuEKF->accb_qy);
	n_p.getParam("AccBiasSTDz", imuEKF->accb_qz);
	n_p.getParam("GyroBiasSTDx", imuEKF->gyrb_qx);
	n_p.getParam("GyroBiasSTDy", imuEKF->gyrb_qy);
	n_p.getParam("GyroBiasSTDz", imuEKF->gyrb_qz);


	n_p.getParam("velSTDx", imuEKF->vel_px);
	n_p.getParam("velSTDy", imuEKF->vel_py);
	n_p.getParam("velSTDz", imuEKF->vel_pz);

	n_p.getParam("KinSTDx", imuEKF->odom_px);
	n_p.getParam("KinSTDy", imuEKF->odom_py);
	n_p.getParam("KinSTDz", imuEKF->odom_pz);
	n_p.getParam("KinSTDOrientx", imuEKF->odom_ax);
	n_p.getParam("KinSTDOrienty", imuEKF->odom_ay);
	n_p.getParam("KinSTDOrientz", imuEKF->odom_az);

	n_p.getParam("VOSTDx", imuEKF->vodom_px);
	n_p.getParam("VOSTDy", imuEKF->vodom_py);
	n_p.getParam("VOSTDz", imuEKF->vodom_pz);
	n_p.getParam("VOSTDOrientx", imuEKF->vodom_ax);
	n_p.getParam("VOSTDOrienty", imuEKF->vodom_ay);
	n_p.getParam("VOSTDOrientz", imuEKF->vodom_az);

	n_p.getParam("gravity", imuEKF->ghat);
	n_p.getParam("gravity", g);


    n_p.param<bool>("calibrateIMUbiases", imuCalibrated, true);
    n_p.param<int>("maxImuCalibrationCycles", maxImuCalibrationCycles, 500);
	if(imuCalibrated)
	{
		//Mahony Filter for Attitude Estimation
		n_p.param<double>("Mahony_Kp", Kp, 0.25);
		n_p.param<double>("Mahony_Ki", Ki, 0.0);
		mh = new Mahony(freq, Kp, Ki);
	}
    std::vector<double> affine_list;

    n_p.getParam("T_B_A", affine_list);
    T_B_A(0, 0) = affine_list[0];
    T_B_A(0, 1) = affine_list[1];
    T_B_A(0, 2) = affine_list[2];
    T_B_A(0, 3) = affine_list[3];
    T_B_A(1, 0) = affine_list[4];
    T_B_A(1, 1) = affine_list[5];
    T_B_A(1, 2) = affine_list[6];
    T_B_A(1, 3) = affine_list[7];
    T_B_A(2, 0) = affine_list[8];
    T_B_A(2, 1) = affine_list[9];
    T_B_A(2, 2) = affine_list[10];
    T_B_A(2, 3) = affine_list[11];
    T_B_A(3, 0) = affine_list[12];
    T_B_A(3, 1) = affine_list[13];
    T_B_A(3, 2) = affine_list[14];
    T_B_A(3, 3) = affine_list[15];

    n_p.getParam("T_B_G", affine_list);
    T_B_G(0, 0) = affine_list[0];
    T_B_G(0, 1) = affine_list[1];
    T_B_G(0, 2) = affine_list[2];
    T_B_G(0, 3) = affine_list[3];
    T_B_G(1, 0) = affine_list[4];
    T_B_G(1, 1) = affine_list[5];
    T_B_G(1, 2) = affine_list[6];
    T_B_G(1, 3) = affine_list[7];
    T_B_G(2, 0) = affine_list[8];
    T_B_G(2, 1) = affine_list[9];
    T_B_G(2, 2) = affine_list[10];
    T_B_G(2, 3) = affine_list[11];
    T_B_G(3, 0) = affine_list[12];
    T_B_G(3, 1) = affine_list[13];
    T_B_G(3, 2) = affine_list[14];
    T_B_G(3, 3) = affine_list[15];

	ROS_INFO_STREAM("IMU EKF Parameters Loaded");

}



void imu_estimator::subscribeToOdom()
{
	odom_sub = n.subscribe(odom_topic,1,&imu_estimator::odomCb,this);
}

void imu_estimator::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_msg = *msg;
	odom_inc = true;
}

void imu_estimator::subscribeToIMU()
{
	imu_sub = n.subscribe(imu_topic,1,&imu_estimator::imuCb,this);
}

void imu_estimator::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_msg = *msg;
	imu_inc = true;
}

void imu_estimator::subscribeToTwist()
{
	twist_sub = n.subscribe(twist_topic,1,&imu_estimator::twistCb,this);
}

void imu_estimator::twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	twist_msg = *msg;
	twist_inc = true;
}




void imu_estimator::subscribeToVO()
{
	vo_sub = n.subscribe(camera_vo_topic,1000,&imu_estimator::voCb,this);
}

void imu_estimator::voCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	vo_msg = *msg;
	vo_inc = true;
}


imu_estimator::imu_estimator() 
{

	useVO = false;
	useOdom = false;
	useTwist = false;

}

imu_estimator::~imu_estimator() {
	if (is_connected_)
		disconnect();
}


void imu_estimator::disconnect() {
	if (!is_connected_)
		return;
	
	is_connected_ = false;
}

bool imu_estimator::connect(const ros::NodeHandle nh) {
	// Initialize ROS nodes
	n = nh;
	// Load ROS Parameters
	loadparams();
	//Initialization
	init();
	// Load IMU parameters
	loadIMUEKFparams();

	dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<imu_ekf::ParamControlConfig> >(n);
    dynamic_reconfigure::Server<imu_ekf::ParamControlConfig>::CallbackType cb = boost::bind(&imu_estimator::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

	// Subscribe/Publish ROS Topics/Services
	subscribe();
	advertise();

	is_connected_ = true;

	ROS_INFO_STREAM("Estimator Initialized");

	return true;
}


bool imu_estimator::connected() {
	return is_connected_;
}

void imu_estimator::subscribe()
{

	subscribeToIMU();

	if (useVO)
		subscribeToVO();
	else if(useOdom)
		subscribeToOdom();
	if(useTwist)
		subscribeToTwist();
	

	sleep(1.0);

}

void imu_estimator::init() {


	// Initialize the IMU based EKF 
	imuEKF = new IMUEKF;
	imuEKF->init();

	imu_inc = false;
	vo_inc = false;
	odom_inc = false;
	twist_inc = false;

    imuCalibrationCycles = 0;
	T_B_A = Affine3d::Identity();
	T_B_G = Affine3d::Identity();
	bias_a = Vector3d::Zero();
	bias_g = Vector3d::Zero();
	Rwb = Matrix3d::Identity();

}

void imu_estimator::reconfigureCB(imu_ekf::ParamControlConfig& config, uint32_t level)
{
      imuEKF->x(9) = config.biasGX;
      imuEKF->x(10) = config.biasGY;
      imuEKF->x(11) = config.biasGZ;
	  imuEKF->x(12) = config.biasAX;
      imuEKF->x(13) = config.biasAY;
      imuEKF->x(14) = config.biasAZ;

      imuEKF->accb_qx = config.AccBiasSTDx;
      imuEKF->accb_qy = config.AccBiasSTDy;
      imuEKF->accb_qz = config.AccBiasSTDz;
      imuEKF->gyrb_qx = config.GyroBiasSTDz;
      imuEKF->gyrb_qy = config.GyroBiasSTDy;
      imuEKF->gyrb_qz = config.GyroBiasSTDx;

      imuEKF->acc_qx = config.AccSTDx;
      imuEKF->acc_qy = config.AccSTDy;
      imuEKF->acc_qz = config.AccSTDz;


      imuEKF->gyr_qx = config.GyroSTDx;
      imuEKF->gyr_qy = config.GyroSTDy;
      imuEKF->gyr_qz = config.GyroSTDz;

      imuEKF->odom_px = config.KinSTDx; 
      imuEKF->odom_py = config.KinSTDy; 
      imuEKF->odom_pz = config.KinSTDz; 

      imuEKF->odom_ax = config.KinSTDOrientx; 
      imuEKF->odom_ay = config.KinSTDOrienty; 
      imuEKF->odom_az = config.KinSTDOrientz; 
}

void imu_estimator::run() {
	static ros::Rate rate(2.0*freq);  //ROS Node Loop Rate
	while (ros::ok()) {
		if(imu_inc)
		{
            if (imuCalibrated)
            {
                mh->updateIMU(T_B_G.linear() * (Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)),
                              T_B_A.linear() * (Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)));
                Rwb = mh->getR();
           
		    
				if(imuCalibrationCycles < maxImuCalibrationCycles)
				{
					bias_g += T_B_G.linear() * Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
					bias_a += T_B_A.linear() * Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z) -  Rwb.transpose() * Vector3d(0,0,g); 
					imuCalibrationCycles++;
					continue;
				}

					biasAX = bias_a(0)/imuCalibrationCycles;
					biasAY = bias_a(1)/imuCalibrationCycles;
					biasAZ = bias_a(2)/imuCalibrationCycles;
					biasGX = bias_g(0)/imuCalibrationCycles;
					biasGY = bias_g(1)/imuCalibrationCycles;
					biasGZ = bias_g(2)/imuCalibrationCycles;
					imuCalibrated = false;
					std::cout<<"Calibration finished at "<<imuCalibrationCycles<<std::endl;
					std::cout<<"Gyro biases "<<biasGX<<" "<<biasGY<<" "<<biasGZ<<std::endl;
					std::cout<<"Acc biases "<<biasAX<<" "<<biasAY<<" "<<biasAZ<<std::endl;
				
			}


			estimateWithIMUEKF();
			//Publish Data
			publishBodyEstimates();
		}
		ros::spinOnce();
		rate.sleep();
	}
	//De-allocation of Heap
	deAllocate();
}

void imu_estimator::estimateWithIMUEKF()
{
		predictWithImu = false;
		//Initialize the IMU EKF state
		if (imuEKF->firstrun) {
			imuEKF->setdt(1.0/freq);
			imuEKF->setBodyPos(Vector3d(0,0,0));
			imuEKF->setBodyOrientation(Matrix3d::Identity());
			imuEKF->setAccBias(Vector3d(biasAX, biasAY, biasAZ));
			imuEKF->setGyroBias(Vector3d(biasGX, biasGY, biasGZ));
			imuEKF->firstrun = false;
		}




		//Compute the attitude and posture with the IMU-Kinematics Fusion
		
		//Predict Step
		//Predict with the IMU gyro and acceleration
		if( !predictWithImu &&!imuEKF->firstrun){
			//std::cout<<"Imu "<<std::endl;
		    imuEKF->predict(T_B_G.linear() * Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z),
                        T_B_A.linear() * Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z));
			imu_inc = false;
			predictWithImu = true;
		}


		//UPDATE STEP
		//Update with the odometry
		if(useOdom)
		{
			if(odom_inc && predictWithImu){
				//std::cout<<"Odom "<<std::endl;
				imuEKF->updateWithOdom(Vector3d(odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z),
				Quaterniond(odom_msg.pose.pose.orientation.w,odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z));
				odom_inc = false;
			}
		}
		else if(useVO)
		{
			if(vo_inc && predictWithImu)
			{

				imuEKF->updateWithVO(Vector3d(vo_msg.pose.position.x,vo_msg.pose.position.y,vo_msg.pose.position.z),
				Quaterniond(vo_msg.pose.orientation.w,vo_msg.pose.orientation.x,vo_msg.pose.orientation.y,vo_msg.pose.orientation.z));
				vo_inc = false;
			}
		}
		else if(useTwist)
		{		
			if(twist_inc && predictWithImu)
			{
				imuEKF->updateWithTwistRotation(Vector3d(twist_msg.twist.linear.x,twist_msg.twist.linear.y,twist_msg.twist.linear.z),
				Quaterniond(imu_msg.orientation.w,imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z));
				twist_inc = false;
			}
		}
		else 
			std::cout<<"No Update"<<std::endl;
			
		
}



void imu_estimator::deAllocate()
{
	delete imuEKF;
}





void imu_estimator::publishBodyEstimates() {
	bodyPose_est_msg.header.stamp = ros::Time::now();
	bodyPose_est_msg.header.frame_id = "odom";
	bodyPose_est_msg.pose.position.x = imuEKF->rX;
	bodyPose_est_msg.pose.position.y = imuEKF->rY;
	bodyPose_est_msg.pose.position.z = imuEKF->rZ;
	bodyPose_est_msg.pose.orientation.x = imuEKF->qib.x();
	bodyPose_est_msg.pose.orientation.y = imuEKF->qib.y();
	bodyPose_est_msg.pose.orientation.z = imuEKF->qib.z();
	bodyPose_est_msg.pose.orientation.w = imuEKF->qib.w();
	bodyPose_est_pub.publish(bodyPose_est_msg);

	bodyVel_est_msg.header.stamp = ros::Time::now();
	bodyVel_est_msg.header.frame_id = "odom";
	bodyVel_est_msg.twist.linear.x = imuEKF->velX;
	bodyVel_est_msg.twist.linear.y = imuEKF->velY;
	bodyVel_est_msg.twist.linear.z = imuEKF->velZ;
	bodyVel_est_msg.twist.angular.x = imuEKF->gyroX;
	bodyVel_est_msg.twist.angular.y = imuEKF->gyroY;
	bodyVel_est_msg.twist.angular.z = imuEKF->gyroZ;
	bodyVel_est_pub.publish(bodyVel_est_msg);

	bodyAcc_est_msg.header.stamp = ros::Time::now();
	bodyAcc_est_msg.header.frame_id = "odom";
	bodyAcc_est_msg.linear_acceleration.x = imuEKF->accX;
	bodyAcc_est_msg.linear_acceleration.y = imuEKF->accY;
	bodyAcc_est_msg.linear_acceleration.z = imuEKF->accZ;
	bodyAcc_est_msg.angular_velocity.x = imuEKF->gyroX;
	bodyAcc_est_msg.angular_velocity.y = imuEKF->gyroY;
	bodyAcc_est_msg.angular_velocity.z = imuEKF->gyroZ;
	bodyAcc_est_pub.publish(bodyAcc_est_msg);


	odom_est_msg.header.stamp=ros::Time::now();
	odom_est_msg.header.frame_id = "odom";
	odom_est_msg.pose.pose = bodyPose_est_msg.pose;
	odom_est_msg.twist.twist = bodyVel_est_msg.twist;
	odom_est_pub.publish(odom_est_msg);


}




void imu_estimator::advertise() {

	bodyPose_est_pub = n.advertise<geometry_msgs::PoseStamped>(
	"imu_ekf/body/pose", 1000);


	bodyVel_est_pub = n.advertise<geometry_msgs::TwistStamped>(
	"imu_ekf/body/vel", 1000);

	bodyAcc_est_pub = n.advertise<sensor_msgs::Imu>(
	"imu_ekf/body/acc", 1000);

	odom_est_pub = n.advertise<nav_msgs::Odometry>("imu_ekf/odom",1000);		
}




