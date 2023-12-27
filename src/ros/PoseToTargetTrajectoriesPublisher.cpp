#include "ros/PoseToTargetTrajectoriesPublisher.h"

namespace ocs2 {

PoseToTargetTrajectoriesPublisher::PoseToTargetTrajectoriesPublisher(
    ::ros::NodeHandle& nodeHandle, 
    const std::string& topicPrefix,
    PoseToTargetTrajectories PoseToTargetTrajectories)
    : poseToTargetTrajectoriesFun_(std::move(PoseToTargetTrajectories)) {

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    newObservationReceived_ = true;
    ROS_INFO("Get latest observation");
  };

  std::string output_topic = topicPrefix + "_mpc_observation";
  ROS_INFO(output_topic.c_str());
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
  ROS_INFO("publisher initialized");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PoseToTargetTrajectoriesPublisher::publishPoseCommand(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  // get the latest observation
  while (ros::ok() && ros::master::check() && !newObservationReceived_) {

    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    if (newObservationReceived_) {
      const auto targetTrajectories = poseToTargetTrajectoriesFun_(position, orientation, observation);
      // publish TargetTrajectories
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }
  }
}

} // namespace ocs2


TargetTrajectories poseToTargetTrajectories(const Eigen::Vector3d& position, 
                                            const Eigen::Quaterniond& orientation,
                                            const SystemObservation& observation) {
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation quaternion
    const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
    const vector_array_t stateTrajectory{target};
    // input trajectory
    const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}


int main(int argc, char** argv) {

    ::ros::init(argc, argv, "mobile_manipulator_pose_publisher");
    ::ros::NodeHandle nodeHandle;

      // sample 
    Eigen::Vector3d position(1.0, 1.0, 1.0); //
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0); // w x y z

    ocs2::PoseToTargetTrajectoriesPublisher publisher(nodeHandle, "mobile_manipulator", &poseToTargetTrajectories);
    publisher.publishPoseCommand(position, orientation);
    return 0;
}