/*
 * imu_ekf
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
 
#ifndef  __IMUEKF_H__
#define  __IMUEKF_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>       /* isnan, sqrt */
/* @author MUMRA
 */

/* @brief  Filters the IMU on X, Y, Z
 * dx/dt=f(x,u,w)
 * y_k=Hx_k+v
 * State is  position in Inertial frame : rX, rY, rZ
 * velocity in  Body frame : velX, velY, velZ
 * orientation of Body frame wrt Inertial frame: angleX, angleY, angleZ
 * accelerometer bias in Body frame : biasAX, biasAY, biasAZ
 * gyro bias in Body frame : biasGx, biasGy, biasGz

 * Measurement is Body Velocity in Inertial frame
 * and the orientation by the Kinematics/Encoders
 */

 
using namespace Eigen;
using namespace std;

class IMUEKF {

private:


	Matrix<double, 15, 12> Lcf;

	Matrix<double, 15, 15> P, Af, Acf, If, Qff;

	Matrix<double, 6, 15> Hf;

	Matrix<double, 12, 12> Qf;

	Matrix<double, 15, 6> Kf;

	//Correction vector
	Matrix<double, 15, 1> dxf;

	//General variables
	Matrix<double, 6, 6> s, R;

	Matrix3d tempM;

	//innovation, position, velocity , acc bias, gyro bias, bias corrected acc, bias corrected gyr, temp vectors
	Vector3d r, v, chi, bf, bw, fhat, omegahat, omega, f, temp;

	Matrix<double, 6, 1> z;
	//Quaternion




public:
	//State vector - with biases included
	Matrix<double, 15, 1> x;

	bool firstrun;
	// Gravity vector
	Vector3d g;

	//Noise Stds

	double AccSTDx, AccSTDy, AccSTDz, GyroSTDx, GyroSTDy, GyroSTDz, KinSTDx,
			KinSTDy, KinSTDz, AccBiasSTDx, AccBiasSTDy, AccBiasSTDz,
			GyroBiasSTDx, GyroBiasSTDy, GyroBiasSTDz, KinSTDOrientx,
			KinSTDOrienty, KinSTDOrientz;

	double gyroX, gyroY, gyroZ, angleX, angleY, angleZ, biasGX, biasGY, biasGZ,
			biasAX, biasAY, biasAZ, ghat;

	double accX, accY, accZ, velX, velY, velZ, rX, rY, rZ;

	Matrix3d Rib, Ris;

	Affine3d Tis, Tib;

	Quaterniond qib_, qis_;

	//Sampling time = 1.0/Sampling Frequency
	double dt;

	void updateVars();

	void setdt(double dtt) {
		dt = dtt;
	}

	void setGyroBias(Vector3d bgyr)
	{
		x(9) = bgyr(0);
		x(10) = bgyr(1);
		x(11) = bgyr(2);
		biasGX = bgyr(0);
		biasGY = bgyr(1);
		biasGZ = bgyr(2);
	}
	void setAccBias(Vector3d bacc)
	{
		x(12) = bacc(0);
		x(13) = bacc(1);
		x(14) = bacc(2);
		biasAX = bacc(0);
		biasAY = bacc(1);
		biasAZ = bacc(2);
	}
	//Initialize the Position
	void setBodyPos(Vector3d bp) {
		x(6) = bp(0);
		x(7) = bp(1);
		x(8) = bp(2);
	}

	//Initialize the Rotation Matrix and the Orientation Error
	void setBodyOrientation(Matrix3d Rot_){
		Rib = Rot_;
	}



	/** @fn void Filter(Matrix<double,3,1> f, Matrix<double,3,1> omega, Matrix<double,3,1>  y_r, Matrix<double,3,1>  y_q)
	 *  @brief filters the acceleration measurements from the IMU
	 */
	void predict(Vector3d omega_, Vector3d f_);
	void updatewithVO(Vector3d y, Quaterniond qy);
	void updateWithOdom(Vector3d y, Quaterniond qy);
	void updateWithTwist(Vector3d y, Quaterniond qy);
	// Initializing Variables
	void init();
	void updateTF();

	//Computes the skew symmetric matrix of a 3-D vector
	inline Matrix3d wedge(
			Vector3d v) {

		Matrix3d skew;

		skew = Matrix3d::Zero();
		skew(0, 1) = -v(2);
		skew(0, 2) = v(1);
		skew(1, 2) = -v(0);
		skew(1, 0) = v(2);
		skew(2, 0) = -v(1);
		skew(2, 1) = v(0);

		return skew;

	}

	//Rodriguez Formula
	inline Matrix<double, 3, 3> expMap(
			Vector3d omega, double theta) {

		Matrix<double, 3, 3> res, omega_skew, I;
		double omeganorm;
		res = Matrix<double, 3, 3>::Zero();
		omega_skew = Matrix<double, 3, 3>::Zero();


		omega *= theta;

		omeganorm = omega.norm();
		I = Matrix<double, 3, 3>::Identity();
		omega_skew = wedge(omega);

		res = I;
		res += omega_skew * (sin(omeganorm) / omeganorm);
		res += (omega_skew * omega_skew) * (
				(1.000 - cos(omeganorm)) / (omeganorm * omeganorm));

		return res;
	}

	inline Vector3d logMap(
			Matrix<double, 3, 3> Rt) {

		Vector3d res;
		res = Vector3d::Zero();

		if (Rt.trace() != 1.000) {
			double theta = acos((Rt.trace() - 1.000) / 2.000);

			double temp = sqrt(
					(double) ((Rt(2, 1) - Rt(1, 2)) * (Rt(2, 1) - Rt(1, 2))
							+ (Rt(0, 2) - Rt(2, 0)) * (Rt(0, 2) - Rt(2, 0))
							+ (Rt(1, 0) - Rt(0, 1)) * (Rt(1, 0) - Rt(0, 1))));

			res(0) = Rt(2, 1) - Rt(1, 2);
			res(1) = Rt(0, 2) - Rt(2, 0);
			res(2) = Rt(1, 0) - Rt(0, 1);
			res *= theta / temp;
		}
		return res;
	}

inline Vector3d logMap(
			Quaterniond q) {

		Vector3d omega;
		omega = Vector3d::Zero();

		double temp = q.norm();

		Vector3d tempV = Vector3d(q.x(), q.y(), q.z());

		double temp_ = tempV.norm();
		tempV *= (1.000 / temp_);


		omega = tempV * (2.0 * acos(q.w() / temp));
		if(isnan(omega(0) + omega(1) + omega(2)))
			omega = Vector3d::Zero();

		return omega;
	}




	
	//Get the Euler Angles from a Rotation Matrix
	inline Vector3d getEulerAngles(
			Matrix3d Rt) {
		Vector3d res;
		res = Vector3d::Zero();

		res(0) = atan2(Rt(2, 1), Rt(2, 2));
		res(1) = atan2(-Rt(2, 0), sqrt(pow(Rt(2, 1), 2) + pow(Rt(2, 2), 2)));
		res(2) = atan2(Rt(1, 0), Rt(0, 0));
		return res;
	}



	inline Matrix3d getRotationMatrix(
			Vector3d angles_) {
		Matrix3d res, Rz, Ry, Rx;
		Rz = Matrix3d::Zero();

		Rz(0, 0) = cos(angles_(2));
		Rz(0, 1) = -sin(angles_(2));
		Rz(1, 0) = sin(angles_(2));
		Rz(1, 1) = cos(angles_(2));
		Rz(2, 2) = 1.000;

		Ry = Matrix3d::Zero();
		Ry(0, 0) = cos(angles_(1));
		Ry(0, 2) = sin(angles_(1));
		Ry(1, 1) = 1.000;
		Ry(2, 0) = -sin(angles_(1));
		Ry(2, 2) = cos(angles_(1));

		Rx = Matrix3d::Zero();
		Rx(0, 0) = 1.00;
		Rx(1, 1) = cos(angles_(0));
		Rx(1, 2) = -sin(angles_(0));
		Rx(2, 1) = sin(angles_(0));
		Rx(2, 2) = cos(angles_(0));

		//YAW PITCH ROLL Convention for right handed counterclowise coordinate systems
		res = Rz * Ry * Rx;

		return res;
	}

	

};
#endif
