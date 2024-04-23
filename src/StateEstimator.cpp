/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file StateEstimator.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @date May 1, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief ROS node to fuse information sources and create an accurate state estimation
 *
 * @details Subscribes to the GPS, IMU, and wheel odometry topics, claculates
 * an estimate of the car's current state using GTSAM, and publishes that data.
 ***********************************************/



#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "StateEstimator/StateEstimator.hpp"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose


// macro for getting the time stamp of a ros message
#define TIME(msg) ( static_cast<double>((msg)->header.stamp.sec) + static_cast<double>((msg)->header.stamp.nanosec) * 1e-9 )

namespace StateEstimator
{
  builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time& rclcpp_time)
  {
      builtin_interfaces::msg::Time builtin_time;
      builtin_time.sec = rclcpp_time.seconds();
      builtin_time.nanosec = rclcpp_time.nanoseconds();
      return builtin_time;
  }

  StateEstimatorNode::StateEstimatorNode() :
    Node("StateEstimatorNode"),
    lastImuT_(0.0),
    lastImuTgps_(0.0),
    maxQSize_(0),
    gpsOptQ_(40),
    imuOptQ_(400),
    odomOptQ_(100),
    gotFirstFix_(false)
  {
    // temporary variables to retrieve parameters
    double accSigma, gyroSigma, initialVelNoise, initialBiasNoiseAcc, initialBiasNoiseGyro, initialRotationNoise,
        carXAngle, carYAngle, carZAngle, sensorX, sensorY, sensorZ, sensorXAngle, sensorYAngle, sensorZAngle,
        gravityMagnitude;

    // declare
    this->declare_parameter<double>("InitialRotationNoise", 1.0);
    this->declare_parameter<double>("InitialVelocityNoise", 0.1);
    this->declare_parameter<double>("InitialBiasNoiseAcc", 1e-1);
    this->declare_parameter<double>("InitialBiasNoiseGyro", 1e-2);
    this->declare_parameter<double>("AccelerometerSigma", 6.0e-2);
    this->declare_parameter<double>("GyroSigma", 2.0e-2);
    this->declare_parameter<double>("AccelBiasSigma", 2.0e-4);
    this->declare_parameter<double>("GyroBiasSigma", 3.0e-5);
    this->declare_parameter<double>("GPSSigma", 0.07);
    this->declare_parameter<double>("SensorTransformX", 0.0);
    this->declare_parameter<double>("SensorTransformY", 0.0);
    this->declare_parameter<double>("SensorTransformZ", 0.0);
    this->declare_parameter<double>("SensorXAngle", 0.0);
    this->declare_parameter<double>("SensorYAngle", 0.0);
    this->declare_parameter<double>("SensorZAngle", 0.0);
    this->declare_parameter<double>("CarXAngle", 0.0);
    this->declare_parameter<double>("CarYAngle", 0.0);
    this->declare_parameter<double>("CarZAngle", 0.0);
    this->declare_parameter<double>("Gravity", 9.8);
    this->declare_parameter<bool>("InvertX", false);
    this->declare_parameter<bool>("InvertY", false);
    this->declare_parameter<bool>("InvertZ", false);
    this->declare_parameter<double>("Imudt", 1.0/200.0);
    // get
    this->get_parameter<double>("InitialRotationNoise", initialRotationNoise);
    this->get_parameter<double>("InitialVelocityNoise", initialVelNoise);
    this->get_parameter<double>("InitialBiasNoiseAcc", initialBiasNoiseAcc);
    this->get_parameter<double>("InitialBiasNoiseGyro", initialBiasNoiseGyro);
    this->get_parameter<double>("AccelerometerSigma", accSigma);
    this->get_parameter<double>("GyroSigma", gyroSigma);
    this->get_parameter<double>("AccelBiasSigma", accelBiasSigma_);
    this->get_parameter<double>("GyroBiasSigma", gyroBiasSigma_);
    this->get_parameter<double>("GPSSigma", gpsSigma_);
    this->get_parameter<double>("SensorTransformX", sensorX);
    this->get_parameter<double>("SensorTransformY", sensorY);
    this->get_parameter<double>("SensorTransformZ", sensorZ);
    this->get_parameter<double>("SensorXAngle", sensorXAngle);
    this->get_parameter<double>("SensorYAngle", sensorYAngle);
    this->get_parameter<double>("SensorZAngle", sensorZAngle);
    this->get_parameter<double>("CarXAngle", carXAngle);
    this->get_parameter<double>("CarYAngle", carYAngle);
    this->get_parameter<double>("CarZAngle", carZAngle);
    this->get_parameter<double>("Gravity", gravityMagnitude);
    this->get_parameter<bool>("InvertX", invertx_);
    this->get_parameter<bool>("InvertY", inverty_);
    this->get_parameter<bool>("InvertZ", invertz_);
    this->get_parameter<double>("Imudt", imuDt_);
    // declare
    this->declare_parameter<double>("GPSX", 0.0);
    this->declare_parameter<double>("GPSY", 0.0);
    this->declare_parameter<double>("GPSZ", 0.0);
    this->declare_parameter<bool>("FixedInitialPose", false);
    this->declare_parameter<double>("initialRoll", 0.0);
    this->declare_parameter<double>("intialPitch", 0.0);
    this->declare_parameter<double>("initialYaw", 0.0);
    this->declare_parameter<bool>("FixedOrigin", false);
    this->declare_parameter<double>("latOrigin", 0.0);
    this->declare_parameter<double>("lonOrigin", 0.0);
    this->declare_parameter<double>("altOrigin", 0.0);
    this->declare_parameter<bool>("UseOdom", false);
    this->declare_parameter<double>("MaxGPSError", 10.0);
    // get
    double gpsx, gpsy, gpsz;
    this->get_parameter<double>("GPSX", gpsx);
    this->get_parameter<double>("GPSY", gpsy);
    this->get_parameter<double>("GPSZ", gpsz);
    bool fixedInitialPose;
    double initialRoll, intialPitch, initialYaw;
    this->get_parameter<bool>("FixedInitialPose", fixedInitialPose);
    this->get_parameter<double>("initialRoll", initialRoll);
    this->get_parameter<double>("intialPitch", intialPitch);
    this->get_parameter<double>("initialYaw", initialYaw);
    double latOrigin, lonOrigin, altOrigin;
    bool fixedOrigin_;
    this->get_parameter<bool>("FixedOrigin", fixedOrigin_);
    this->get_parameter<double>("latOrigin", latOrigin);
    this->get_parameter<double>("lonOrigin", lonOrigin);
    this->get_parameter<double>("altOrigin", altOrigin);
    bool usingOdom_;
    double maxGPSError_;
    this->get_parameter<bool>("UseOdom", usingOdom_);
    this->get_parameter<double>("MaxGPSError", maxGPSError_);
    imuPgps_ = Pose3(Rot3(), Point3(gpsx, gpsy, gpsz));
    imuPgps_.print("IMU->GPS");
    if (fixedOrigin_) {
        enu_.Reset(latOrigin, lonOrigin, altOrigin);
    }
    
    std::cout << "InitialRotationNoise " << initialRotationNoise << "\n"
    << "InitialVelocityNoise " << initialVelNoise << "\n"
    << "InitialBiasNoiseAcc " << initialBiasNoiseAcc << "\n"
    << "InitialBiasNoiseGyro " << initialBiasNoiseGyro << "\n"
    << "AccelerometerSigma " << accSigma << "\n"
    << "GyroSigma " << gyroSigma << "\n"
    << "AccelBiasSigma " << accelBiasSigma_ << "\n"
    << "GyroBiasSigma " << gyroBiasSigma_ << "\n"
    << "GPSSigma " << gpsSigma_ << "\n"
    << "SensorTransformX " << sensorX << "\n"
    << "SensorTransformY " << sensorY << "\n"
    << "SensorTransformZ " << sensorZ << "\n"
    << "SensorXAngle " <<  sensorXAngle << "\n"
    << "SensorYAngle " << sensorYAngle << "\n"
    << "SensorZAngle " <<   sensorZAngle << "\n"
    << "CarXAngle " <<  carXAngle << "\n"
    << "CarYAngle " <<  carYAngle << "\n"
    << "CarZAngle " <<  carZAngle << "\n"
    << "Gravity " <<   gravityMagnitude << "\n";

    // Use an ENU frame
    preintegrationParams_ =  PreintegrationParams::MakeSharedU(gravityMagnitude);
    preintegrationParams_->accelerometerCovariance = accSigma * I_3x3;
    preintegrationParams_->gyroscopeCovariance = gyroSigma * I_3x3;
    preintegrationParams_->integrationCovariance = 1e-5 * I_3x3;

    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    optimizedBias_ = imuBias::ConstantBias(biases);
    previousBias_ = imuBias::ConstantBias(biases);
    imuPredictor_ = std::make_shared<PreintegratedImuMeasurements>(preintegrationParams_, optimizedBias_);

    optimizedTime_ = 0;

    //TODO: we need to have a better intialization
    vectornav_msgs::msg::ImuGroup::SharedPtr ip = nullptr;
    // if (!fixedInitialPose)
    // {
    //   while (!ip)
    //   {
    //     RCLCPP_WARN(this->get_logger(),"Waiting for valid initial orientation ...");
    //     ip = rclcpp::wait_for_message<vectornav_msgs::msg::ImuGroup>("/vectornav/raw/imu", this, std::chrono::seconds(5));
    //   }
    //   Rot3 initialRotation = Rot3::Ypr(ip->angularrate.z, ip->angularrate.y, ip->angularrate.x);
    //   initialPose_.orientation.w = initialRotation.quaternion()[0];
    //   initialPose_.orientation.x = initialRotation.quaternion()[1];
    //   initialPose_.orientation.y = initialRotation.quaternion()[2];
    //   initialPose_.orientation.z = initialRotation.quaternion()[3];
    //   initialPose_.bias.x = ip->accel.x;
    //   initialPose_.bias.y = ip->accel.y;
    //   initialPose_.bias.z = ip->accel.z;
    // }
    // else
    {
      RCLCPP_WARN(this->get_logger(), "Using fixed initial orientation");
      Rot3 initialRotation = Rot3::Ypr(initialYaw, intialPitch, initialRoll);
      initialPose_.orientation.w = initialRotation.toQuaternion().w();
      initialPose_.orientation.x = initialRotation.toQuaternion().x();
      initialPose_.orientation.y = initialRotation.toQuaternion().y();
      initialPose_.orientation.z = initialRotation.toQuaternion().z();
      initialPose_.bias.x = 0;
      initialPose_.bias.y = 0;
      initialPose_.bias.z = 0;
    }

    Rot3 initRot(Quaternion(initialPose_.orientation.w, initialPose_.orientation.x, initialPose_.orientation.y,
          initialPose_.orientation.z));

    bodyPSensor_ = Pose3(Rot3::RzRyRx(sensorXAngle, sensorYAngle, sensorZAngle),
        Point3(sensorX, sensorY, sensorZ));
    carENUPcarNED_ = Pose3(Rot3::RzRyRx(carXAngle, carYAngle, carZAngle), Point3());

    bodyPSensor_.print("Body pose\n");
    carENUPcarNED_.print("CarBodyPose\n");

    posePub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pose", 1);
    biasAccPub_ = this->create_publisher<geometry_msgs::msg::Point>("/bias_acc", 1);
    biasGyroPub_ = this->create_publisher<geometry_msgs::msg::Point>("/bias_gyro", 1);
    timePub_ = this->create_publisher<geometry_msgs::msg::Point>("/time_delays", 1);
    // statusPub_ = this->create_publisher<autorally_msgs::msg::StateEstimatorStatus>("status", 1);
    gpsPosPub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps_pose", 1);

    ISAM2Params params;
    params.factorization = ISAM2Params::QR;
    isam_ = new ISAM2(params);

    // prior on the first pose
    priorNoisePose_ = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialRotationNoise, initialRotationNoise, 3*initialRotationNoise,
             gpsSigma_, gpsSigma_, gpsSigma_).finished());

     // Add velocity prior
    priorNoiseVel_ = noiseModel::Diagonal::Sigmas(
         (Vector(3) << initialVelNoise, initialVelNoise, initialVelNoise).finished());

     // Add bias prior
    priorNoiseBias_ = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro).finished());

    std::cout<<"checkpoint"<<std::endl;

    Vector sigma_acc_bias_c(3), sigma_gyro_bias_c(3);
    sigma_acc_bias_c << accelBiasSigma_,  accelBiasSigma_,  accelBiasSigma_;
    sigma_gyro_bias_c << gyroBiasSigma_, gyroBiasSigma_, gyroBiasSigma_;
    noiseModelBetweenBias_sigma_ = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();

    gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 300, std::bind(&StateEstimatorNode::GpsCallback, this, std::placeholders::_1));

    imuSub_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
            "imu", 600, std::bind(&StateEstimatorNode::ImuCallback, this, std::placeholders::_1));

    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", 300, std::bind(&StateEstimatorNode::WheelOdomCallback, this, std::placeholders::_1));
    RCLCPP_WARN(this->get_logger(), "Before thread");
    std::thread optimizer(&StateEstimatorNode::GpsHelper,this);
  }

  StateEstimatorNode::~StateEstimatorNode()
  {}

  void StateEstimatorNode::GpsCallback(sensor_msgs::msg::NavSatFix::ConstPtr fix)
  {
    if (!gpsOptQ_.pushNonBlocking(fix))
      RCLCPP_WARN(this->get_logger(), "Dropping a GPS measurement due to full queue!!");
  }

  void StateEstimatorNode::GetAccGyro(vectornav_msgs::msg::ImuGroup::ConstPtr imu, Vector3 &acc, Vector3 &gyro)
  {
    double accx, accy, accz;
    if (invertx_) accx = -imu->accel.x;
    else accx = imu->accel.x;
    if (inverty_) accy = -imu->accel.y;
    else accy = imu->accel.y;
    if (invertz_) accz = -imu->accel.z;
    else accz = imu->accel.z;
    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;
    if (invertx_) gx = -imu->angularrate.x;
    else gx = imu->angularrate.x;
    if (inverty_) gy = -imu->angularrate.y;
    else gy = imu->angularrate.y;
    if (invertz_) gz = -imu->angularrate.z;
    else gz = imu->angularrate.z;

    gyro = Vector3(gx, gy, gz);
  }


  void StateEstimatorNode::GpsHelper()
  {
    RCLCPP_WARN(this->get_logger(), "In thread");
    rclcpp::Rate loop_rate(10);
    RCLCPP_WARN(this->get_logger(), "In thread1");
    bool gotFirstFix = false;
    double startTime;
    int odomKey = 1;
    int imuKey = 1;
    int latestGPSKey = 0;
    imuBias::ConstantBias prevBias;
    Vector3 prevVel = (Vector(3) << 0.0,0.0,0.0).finished();
    Pose3 prevPose;
    //unsigned char status = autorally_msgs::msg::stateEstimatorStatus::OK;


    while (rclcpp::ok())
    {
      bool optimize = false;

      if (!gotFirstFix)
      {RCLCPP_WARN(this->get_logger(), "In thread2");
        sensor_msgs::msg::NavSatFix::ConstPtr fix = gpsOptQ_.popBlocking();
        startTime = TIME(fix);
        if(imuOptQ_.size() <= 0) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000 /*ms*/, "no IMU messages before first fix, continuing until there is");
          continue;
        }
        // errors out if the IMU and GPS are not close in timestamps
        double most_recent_imu_time = static_cast<double>(imuOptQ_.back()->header.stamp.sec) + static_cast<double>(imuOptQ_.back()->header.stamp.nanosec) * 1e-9;
        if(std::abs(most_recent_imu_time - startTime) > 0.1) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "There is a very large difference in the GPS and IMU timestamps " << most_recent_imu_time - startTime);
          exit(-1);
        }

        if(usingOdom_) {
          lastOdom_ = odomOptQ_.popBlocking();
        }

        NonlinearFactorGraph newFactors;
        Values newVariables;
        gotFirstFix = true;

        double E, N, U;
        if (!fixedOrigin_)
        {
          enu_.Reset(fix->latitude, fix->longitude, fix->altitude);
          E = 0; N = 0; U = 0; // we're choosing this as the origin
        }
        else
        {
          // we are given an origin
          enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
        }

        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(initialPose_.orientation.w,
            initialPose_.orientation.x,
            initialPose_.orientation.y,
            initialPose_.orientation.z);
        std::cout << "Initial orientation" << std::endl;
        std::cout << bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation() << std::endl;
        Pose3 x0(bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation(),
            Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose_);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel_);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, initialPose_.bias.x,
            -initialPose_.bias.y, -initialPose_.bias.z).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias_);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), imuPgps_,
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(imuPgps_));

        isam_->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        lastIMU_ = imuOptQ_.popBlocking();
        //If we only pop one, we need some dt
        lastImuTgps_ = TIME(lastIMU_) - 0.005;
        while(TIME(lastIMU_) < TIME(fix))
        {
          lastImuTgps_ = TIME(lastIMU_);
          lastIMU_ = imuOptQ_.popBlocking();
        }
        loop_rate.sleep();
      }
      else
      {
        NonlinearFactorGraph newFactors;
        Values newVariables;


        // add IMU measurements
        while (imuOptQ_.size() > 0 && (TIME(imuOptQ_.back()) > (startTime + imuKey * 0.1)))
        {
          double curTime = startTime + imuKey * 0.1;
          PreintegratedImuMeasurements pre_int_data(preintegrationParams_, previousBias_);
          while(TIME(lastIMU_) < curTime)
          {
            Vector3 acc, gyro;
            GetAccGyro(lastIMU_, acc, gyro);
            double imuDT = TIME(lastIMU_) - lastImuTgps_;
            lastImuTgps_ = TIME(lastIMU_);
            pre_int_data.integrateMeasurement(acc, gyro, imuDT);
            lastIMU_ = imuOptQ_.popBlocking();
          }
          // adding the integrated IMU measurements to the factor graph
          ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
          newFactors.add(imuFactor);
          newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
              noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenBias_sigma_)));

          // Predict forward to get an initial estimate for the pose and velocity
          NavState curNavState(prevPose, prevVel);
          NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
          newVariables.insert(X(imuKey), nextNavState.pose());
          newVariables.insert(V(imuKey), nextNavState.v());
          newVariables.insert(B(imuKey), previousBias_);
          newVariables.insert(G(imuKey), nextNavState.pose().compose(imuPgps_));
          prevPose = nextNavState.pose();
          prevVel = nextNavState.v();
          ++imuKey;
          optimize = true;
        }


        // add GPS measurements that are not ahead of the imu messages
        while (optimize && gpsOptQ_.size() > 0 && TIME(gpsOptQ_.front()) < (startTime + (imuKey-1)*0.1 + 1e-2))
        {
          sensor_msgs::msg::NavSatFix::ConstPtr fix = gpsOptQ_.popBlocking();
          double timeDiff = (TIME(fix) - startTime) / 0.1;
          int key = round(timeDiff);
          if (std::abs(timeDiff - key) < 1e-4)
          {
            // this is a gps message for a factor
            latestGPSKey = key;
            double E,N,U;
            enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);

            // check if the GPS message is close to our expected position
            Pose3 expectedState;
            if (newVariables.exists(X(key)))
              expectedState = (Pose3) newVariables.at<Pose3>(X(key));
            else
              expectedState = isam_->calculateEstimate<Pose3>(X(key));

            double dist = std::sqrt( std::pow(expectedState.x() - E, 2) + std::pow(expectedState.y() - N, 2) );
            if (dist < maxGPSError_ || latestGPSKey < imuKey-2)
            {
              geometry_msgs::msg::PoseWithCovarianceStamped point;
              point.header.stamp = toBuiltinTime(rclcpp::Clock(RCL_ROS_TIME).now());//TODO: use now or bag time?
              point.header.frame_id = "odom";
              point.pose.pose.position.x = E;
              point.pose.pose.position.y = N;
              point.pose.covariance[0] = fix->position_covariance[0];
              point.pose.covariance[7] = fix->position_covariance[4];
              gpsPosPub_->publish(point);

              SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(gpsSigma_, gpsSigma_, 3.0 * gpsSigma_));
              GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
              newFactors.add(gpsFactor);
              BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), imuPgps_,
                  noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
              newFactors.add(imuPgpsFactor);

              if (!usingOdom_)
                odomKey = key+1;
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), "Received bad GPS message");
            }
          }
        }



        // if only using odom with no GPS, then remove old messages from queue
        while (!usingOdom_ && odomOptQ_.size() > 0 && TIME(odomOptQ_.front()) < (odomKey*0.1 + startTime))
          lastOdom_ = odomOptQ_.popBlocking();

        // if available, add any odom factors that are not ahead of the imu messages
        while ((usingOdom_ || latestGPSKey < imuKey-2) && optimize && odomKey < imuKey && odomOptQ_.size() > 0
            && (TIME(odomOptQ_.back()) > (startTime + odomKey * 0.1)))
        {
          double prevTime = startTime + (odomKey-1) * 0.1;
          newFactors.add(integrateWheelOdom(prevTime, prevTime+0.1, odomKey++));
        }


        // if we processed imu - then we can optimize the state
        if (optimize)
        {
          try
          {
            isam_->update(newFactors, newVariables);
            Pose3 nextState = isam_->calculateEstimate<Pose3>(X(imuKey-1));

            prevPose = nextState;
            prevVel = isam_->calculateEstimate<Vector3>(V(imuKey-1));
            prevBias = isam_->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

            double curTime = startTime + (imuKey-1) * 0.1;

            {
              std::lock_guard guard(optimizedStateMutex_);
              optimizedState_ = NavState(prevPose, prevVel);
              optimizedBias_ = prevBias;
              optimizedTime_ = curTime;
              //status_ = status;
            }

            nav_msgs::msg::Odometry poseNew;
            poseNew.header.stamp = toBuiltinTime(rclcpp::Time(curTime));//TODO

            geometry_msgs::msg::Point ptAcc;
            ptAcc.x = prevBias.vector()[0];
            ptAcc.y = prevBias.vector()[1];
            ptAcc.z = prevBias.vector()[2];

            geometry_msgs::msg::Point ptGyro;
            ptGyro.x = prevBias.vector()[3];
            ptGyro.y = prevBias.vector()[4];
            ptGyro.z = prevBias.vector()[5];

            biasAccPub_->publish(ptAcc);
            biasGyroPub_->publish(ptGyro);
          }
          catch(gtsam::IndeterminantLinearSystemException ex)
          {
            RCLCPP_ERROR(this->get_logger(), "Encountered Indeterminant System Error!");
            //status = autorally_msgs::msg::stateEstimatorStatus::ERROR;
            {
              std::lock_guard guard(optimizedStateMutex_);
              //status_ = status;
            }
          }
        }
        loop_rate.sleep();
      }
    }
  }


  void StateEstimatorNode::ImuCallback(vectornav_msgs::msg::ImuGroup::SharedPtr imu)
  {
    double dt;
    if (lastImuT_ == 0) dt = 0.005;
    else dt = TIME(imu) - lastImuT_;

    lastImuT_ = TIME(imu);
    //ros::Time before = ros::Time::now();

    // Push the IMU measurement to the optimization thread
    int qSize = imuOptQ_.size();
    if (qSize > maxQSize_)
      maxQSize_ = qSize;
    if (!imuOptQ_.pushNonBlocking(imu))
      RCLCPP_WARN(this->get_logger(), "Dropping an IMU measurement due to full queue!!");

    // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
    imuMeasurements_.push_back(imu);
    //Grab the most current optimized state
    double optimizedTime;
    NavState optimizedState;
    imuBias::ConstantBias optimizedBias;
    //unsigned char status;
    {
      std::lock_guard guard(optimizedStateMutex_);
      optimizedState = optimizedState_;
      optimizedBias = optimizedBias_;
      optimizedTime = optimizedTime_;
      //status = status_;
    }
    if (optimizedTime == 0) return; // haven't optimized first state yet

    bool newMeasurements = false;
    int numImuDiscarded = 0;
    double imuQPrevTime;
    Vector3 acc, gyro;
    while (!imuMeasurements_.empty() && (TIME(imuMeasurements_.front()) < optimizedTime))
    {
      imuQPrevTime = TIME(imuMeasurements_.front());
      imuMeasurements_.pop_front();
      newMeasurements = true;
      numImuDiscarded++;
    }

    if(newMeasurements)
    {
      //We need to reset integration and iterate through all our IMU measurements
      imuPredictor_->resetIntegration();
      int numMeasurements = 0;
      for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it)
      {
        double dt_temp =  TIME(*it) - imuQPrevTime;
        imuQPrevTime = TIME(*it);
        GetAccGyro(*it, acc, gyro);
        imuPredictor_->integrateMeasurement(acc, gyro, dt_temp);
        numMeasurements++;
        // ROS_INFO("IMU time %f, dt %f", (*it)->header.stamp.toSec(), dt_temp);
      }
      // ROS_INFO("Resetting Integration, %d measurements integrated, %d discarded", numMeasurements, numImuDiscarded);
    }
    else
    {
      //Just need to add the newest measurement, no new optimized pose
      GetAccGyro(imu, acc, gyro);
      imuPredictor_->integrateMeasurement(acc, gyro, dt);
      // ROS_INFO("Integrating %f, dt %f", m_lastImuT, dt);
    }

    // predict next state given the imu measurements
    NavState currentPose = imuPredictor_->predict(optimizedState, optimizedBias);
    nav_msgs::msg::Odometry poseNew;
    poseNew.header.stamp = toBuiltinTime(imu->header.stamp);

    Vector4 q = currentPose.quaternion().coeffs();
    poseNew.pose.pose.orientation.x = q[0];
    poseNew.pose.pose.orientation.y = q[1];
    poseNew.pose.pose.orientation.z = q[2];
    poseNew.pose.pose.orientation.w = q[3];

    poseNew.pose.pose.position.x = currentPose.position().x();
    poseNew.pose.pose.position.y = currentPose.position().y();
    poseNew.pose.pose.position.z = currentPose.position().z();

    poseNew.twist.twist.linear.x = currentPose.velocity().x();
    poseNew.twist.twist.linear.y = currentPose.velocity().y();
    poseNew.twist.twist.linear.z = currentPose.velocity().z();
    
    poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
    poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
    poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

    poseNew.child_frame_id = "base_link";
    poseNew.header.frame_id = "odom";

    posePub_->publish(poseNew);

    geometry_msgs::msg::Point delays;
    delays.x = TIME(imu);
    delays.y = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - (static_cast<double>(imu->header.stamp.sec)+static_cast<double>(imu->header.stamp.nanosec)*1e-9);//TODO
    delays.z = TIME(imu) - optimizedTime;
    timePub_->publish(delays);

    // // publish the status of the estimate - set in the gpsHelper thread
    // autorally_msgs::msg::stateEstimatorStatus statusMsgs;
    // statusMsgs.header.stamp = imu->header.stamp;
    // statusMsgs.status = status;
    // statusPub_.publish(statusMsgs);
    return;
  }

  void StateEstimatorNode::WheelOdomCallback(nav_msgs::msg::Odometry::ConstPtr odom)
  {
    if (!odomOptQ_.pushNonBlocking(odom) && usingOdom_) {
      RCLCPP_WARN(this->get_logger(), "Dropping an wheel odometry measurement due to full queue!!");
    }
  }


  BetweenFactor<Pose3> StateEstimatorNode::integrateWheelOdom(double prevTime, double stopTime, int curKey)
  {
    double x=0, y=0, theta=0, xVar=0, yVar=0, zVar=0, thetaVariance=0, dt=0, lastTimeUsed=prevTime;

    while (lastTimeUsed != stopTime)
    {
      if (odomOptQ_.size() != 0 && TIME(odomOptQ_.front()) < stopTime)
      {
        lastOdom_ = odomOptQ_.popBlocking();
        dt = TIME(lastOdom_) - lastTimeUsed;
        lastTimeUsed = TIME(lastOdom_);
      }
      else
      {
        dt = stopTime - lastTimeUsed;
        lastTimeUsed = stopTime;
      }

      // the local frame velocities
      double vx = lastOdom_->twist.twist.linear.x;
      double vy = lastOdom_->twist.twist.linear.y;
      // update the relative position from the initial
      x += vx*dt*cos(theta) - vy*dt*sin(theta);
      y += vx*dt*sin(theta) + vy*dt*cos(theta);
      theta += dt*lastOdom_->twist.twist.angular.z;
      xVar += dt * lastOdom_->twist.covariance[0];
      yVar += dt * lastOdom_->twist.covariance[7];
      zVar += dt * lastOdom_->twist.covariance[14];
      thetaVariance += dt*lastOdom_->twist.covariance[35];
    }

    Pose3 betweenPose = Pose3(Rot3::Rz(theta), Point3(x, y, 0.0));
    return BetweenFactor<Pose3>(X(curKey-1), X(curKey), betweenPose, noiseModel::Diagonal::Sigmas(
          (Vector(6) << thetaVariance*2,thetaVariance*2,thetaVariance,xVar,yVar,zVar).finished()));
  }

};