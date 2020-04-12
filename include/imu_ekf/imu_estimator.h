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

#ifndef IMUESTIMATOR_H
#define IMUESTIMATOR_H

// ROS Headers
#include <ros/ros.h>

// Estimator Headers
#include "imu_ekf/IMUEKF.h"

#include <eigen3/Eigen/Dense>
// ROS Messages
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <imu_ekf/ParamControlConfig.h>

#include <imu_ekf/Mahony.h>


using namespace Eigen;
using namespace std;

class imu_estimator{
private:
	// ROS Standard Variables
	ros::NodeHandle n;

	ros::Publisher bodyPose_est_pub, bodyVel_est_pub, bodyAcc_est_pub, odom_est_pub;
    ros::Subscriber imu_sub, vo_sub, odom_sub, twist_sub;
	
	double  freq;
	bool vo_inc, imu_inc, joint_inc, odom_inc, twist_inc;
	bool predictWithImu;
    boost::shared_ptr< dynamic_reconfigure::Server<imu_ekf::ParamControlConfig> > dynamic_recfg_;


	//ROS Messages
	
	sensor_msgs::Imu imu_msg;
	nav_msgs::Odometry odom_msg, odom_est_msg;
	geometry_msgs::PoseStamped vo_msg;
   
    geometry_msgs::PoseStamped bodyPose_est_msg;
	geometry_msgs::TwistStamped bodyVel_est_msg, twist_msg;
	sensor_msgs::Imu  bodyAcc_est_msg;

	Mahony* mh;
	// Helper
	bool is_connected_;

	double g;
	Vector3d bias_g, bias_a;
	IMUEKF* imuEKF;
	Matrix3d Rwb;
	double Kp, Ki; //Mahony gains

	bool firstrun;
	int maxImuCalibrationCycles, imuCalibrationCycles;
	bool imuCalibrated;
	double biasAX,biasAY,biasAZ,biasGX,biasGY,biasGZ;
    Affine3d T_B_G, T_B_A;
	 string camera_vo_topic;
	 string imu_topic;
	 string odom_topic;
	 string twist_topic;
	 bool useVO, useOdom, useTwist;

	//Odometry, from supportleg to inertial, transformation from support leg to other leg
     void subscribeToIMU();
 	 void subscribeToVO();
	 void subscribeToOdom();
	 void subscribeToTwist();

	 void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
	 void voCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	 void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
	 void twistCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	 void reconfigureCB(imu_ekf::ParamControlConfig& config, uint32_t level);

	//private methods
	void init();

	void estimateWithIMUEKF();


	void deAllocate();
	void publishBodyEstimates();
	// Advertise to ROS Topics
	void advertise();
	void subscribe();

public:




	// Constructor/Destructor
	imu_estimator();

	~imu_estimator();

	// Connect/Disconnet to ALProxies
	bool connect(const ros::NodeHandle nh);

	void disconnect();



	// Parameter Server
	void loadparams();
	void loadIMUEKFparams();
	// General Methods

	void run();

	bool connected();


};

#endif // IMUESTIMATOR_H
