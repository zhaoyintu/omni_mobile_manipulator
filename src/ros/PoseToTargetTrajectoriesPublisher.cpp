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

    // parser command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("position") == 0) {
            position.x() = (i + 1 < argc) ? std::atof(argv[++i]) : 0.0;
            position.y() = (i + 1 < argc) ? std::atof(argv[++i]) : 0.0;
            position.z() = (i + 1 < argc) ? std::atof(argv[++i]) : 1.0;
        } else if (arg.find("orientation") == 0) {
            orientation.w() = (i + 1 < argc) ? std::atof(argv[++i]) : 1.0;
            orientation.x() = (i + 1 < argc) ? std::atof(argv[++i]) : 0.0;
            orientation.y() = (i + 1 < argc) ? std::atof(argv[++i]) : 0.0;
            orientation.z() = (i + 1 < argc) ? std::atof(argv[++i]) : 0.0;
        }
    }

    ROS_INFO("position: %f %f %f", position.x(), position.y(), position.z());
    ROS_INFO("orientation: %f %f %f %f", orientation.w(), orientation.x(), orientation.y(), orientation.z());

    // check if orientation is valid
    if (std::abs(orientation.norm() - 1.0) > 1e-6) {
        ROS_ERROR("Invalid orientation: %f %f %f %f", orientation.w(), orientation.x(), orientation.y(), orientation.z());
        return -1;
    }

    ocs2::PoseToTargetTrajectoriesPublisher publisher(nodeHandle, "mobile_manipulator", &poseToTargetTrajectories);
    publisher.publishPoseCommand(position, orientation);
    return 0;
}