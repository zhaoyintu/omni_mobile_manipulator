#include "ros/PoseToTargetTrajectoriesPublisher.h"

// namespace ocs2 {

PoseToTargetTrajectoriesPublisher::PoseToTargetTrajectoriesPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         PoseToTargetTrajectories PoseToTargetTrajectories)
    : poseToTargetTrajectoriesFun_(std::move(PoseToTargetTrajectories)) {

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PoseToTargetTrajectoriesPublisher::publishPoseCommand(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  // get TargetTrajectories
  const auto targetTrajectories = poseToTargetTrajectoriesFun_(position, orientation, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

// } // namespace ocs2


TargetTrajectories poseToTargetTrajectories(const Eigen::Vector3d& position, 
                                            const Eigen::Quaterniond& orientation,
                                            const SystemObservation& observation) {
    // ocs2::scalar_array_t timeTrajectory;
    // timeTrajectory.push_back(observation.time);

    // ocs2::vector_array_t stateTrajectory;
    // Eigen::VectorXd state(7); 
    // state << position, orientation.coeffs();
    // stateTrajectory.push_back(state);

    // ocs2::vector_array_t inputTrajectory;
    // inputTrajectory.push_back(Eigen::VectorXd::Zero(observation.input.size()));

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

    ocs2::PoseToTargetTrajectoriesPublisher publisher(nodeHandle, "mobile_manipulator", &poseToTargetTrajectories);

    // sample 
    Eigen::Vector3d position(1.0, 0.0, 0.0); //
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0); // w x y z
    ros::spinOnce();
    // publish
    publisher.publishPoseCommand(position, orientation);
    return 0;
}