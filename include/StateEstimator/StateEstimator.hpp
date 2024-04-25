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
 * @file StateEstimator.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date May 1, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief StateEstimator class definition
 *
 ***********************************************/

#ifndef StateEstimator_H_
#define StateEstimator_H_

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include <list>
#include <iostream>
#include <fstream>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"

#include "StateEstimator/BlockingQueue.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define PI 3.14159265358979323846264338


namespace StateEstimator
{   
    struct Orientation_{
        double w{0};
        double x{0};
        double y{0};
        double z{0};
    };
    struct Bias_{
        double x{0};
        double y{0};
        double z{0};
    };
    struct IMU_state_ {
        Orientation_ orientation;
        Bias_ bias;
    };


  class StateEstimatorNode : public rclcpp::Node
  {
  private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;//gps/fix
    rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr imuSub_;///vectornav/raw/imu
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;//dont care
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasAccPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasGyroPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr timePub_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gpsPosPub_;


    double lastImuT_, lastImuTgps_;
    //unsigned char status_;
    double accelBiasSigma_, gyroBiasSigma_;
    double gpsSigma_;
    int maxQSize_;

    BlockingQueue<sensor_msgs::msg::NavSatFix::ConstPtr> gpsOptQ_;
    //BlockingQueue<sensor_msgs::ImuConstPtr> imuOptQ_;
    BlockingQueue<std::shared_ptr<vectornav_msgs::msg::ImuGroup>> imuOptQ_;
    BlockingQueue<nav_msgs::msg::Odometry::ConstPtr> odomOptQ_;

    std::mutex optimizedStateMutex_;
    gtsam::NavState optimizedState_;
    double optimizedTime_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPredictor_;
    double imuDt_;
    gtsam::imuBias::ConstantBias optimizedBias_, previousBias_;
    //sensor_msgs::ImuConstPtr lastIMU_;
    vectornav_msgs::msg::ImuGroup::SharedPtr lastIMU_;
    std::shared_ptr<gtsam::PreintegrationParams> preintegrationParams_;

    std::list<std::shared_ptr<vectornav_msgs::msg::ImuGroup>> imuMeasurements_, imuGrav_;
    IMU_state_ initialPose_;//TODO
    gtsam::Pose3 bodyPSensor_, carENUPcarNED_;
    gtsam::Pose3 imuPgps_;

    bool fixedOrigin_;
    GeographicLib::LocalCartesian enu_;// Object to put lat/lon coordinates into local cartesian
    bool gotFirstFix_;
    bool invertx_, inverty_, invertz_;
    bool usingOdom_;
    double maxGPSError_;

    gtsam::SharedDiagonal priorNoisePose_;
    gtsam::SharedDiagonal priorNoiseVel_;
    gtsam::SharedDiagonal priorNoiseBias_;
    gtsam::Vector noiseModelBetweenBias_sigma_;
    gtsam::ISAM2 *isam_;

    nav_msgs::msg::Odometry::ConstPtr lastOdom_;

    std::thread optimizer;

  public:
    StateEstimatorNode();
    ~StateEstimatorNode();
    void GpsCallback(sensor_msgs::msg::NavSatFix::ConstPtr fix);
    void ImuCallback(vectornav_msgs::msg::ImuGroup::SharedPtr imu);
    void WheelOdomCallback(nav_msgs::msg::Odometry::ConstPtr odom);
    void GpsHelper();
    gtsam::BetweenFactor<gtsam::Pose3> integrateWheelOdom(double prevTime, double stopTime, int curFactor);
    void GetAccGyro(vectornav_msgs::msg::ImuGroup::ConstPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
  };
  builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time& rclcpp_time);
};

#endif /* StateEstimator_H_ */