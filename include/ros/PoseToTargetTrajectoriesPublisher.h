#ifndef POSE_TO_TARGET_TRAJECTORIES_PUBLISHER_H
#define POSE_TO_TARGET_TRAJECTORIES_PUBLISHER_H

#include <functional>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

using namespace ocs2;

namespace ocs2 {

class PoseToTargetTrajectoriesPublisher {
    public:
        using PoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const SystemObservation& observation)>;
            
        PoseToTargetTrajectoriesPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                    PoseToTargetTrajectories poseToTargetTrajectoriesFun);

        void publishPoseCommand(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    private:
        PoseToTargetTrajectories poseToTargetTrajectoriesFun_;

        std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
        ::ros::Subscriber observationSubscriber_;
        mutable std::mutex latestObservationMutex_;
        SystemObservation latestObservation_;

        bool newObservationReceived_ = false;
};

} // namespace ocs2

#endif // POSE_TO_TARGET_TRAJECTORIES_PUBLISHER_H
