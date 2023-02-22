#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


namespace ROS2
{
    enum class GoalStatus
    {
        Pending,
        Active,
        Concluded
    };

    class FollowJointTrajectoryActionServer
    {
    public:
        using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        FollowJointTrajectoryActionServer() = default;
        void CreateServer(AZStd::string ROS2ControllerName);

        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr m_actionServer;
        std::shared_ptr<GoalHandleFollowJointTrajectory> m_goalHandle;

        GoalStatus m_goalStatus = GoalStatus::Pending;

    protected:
        // callbacks for action_server_
        rclcpp_action::GoalResponse goal_received_callback(
            const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse goal_cancelled_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void goal_accepted_callback(
            std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    };

    //! Component responsible for controlling a robotic arm made up of hinge joints.
    class ManipulatorControllerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        enum class Controller
        {
            FeedForward, //!< @see <a href="https://en.wikipedia.org/wiki/Feed_forward_(control)">FeedForward</a>.
            PID          //!< @see <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>.
        };

        AZ_COMPONENT(ManipulatorControllerComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component); // , ManipulatorRequestBus::Handler);

        ManipulatorControllerComponent() = default;
        ~ManipulatorControllerComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void InitializeMap();
        void InitializePid();
        void InitializeCurrentPosition();
        void KeepStillPosition(const uint64_t deltaTimeNs);
        void ExecuteTrajectory(const uint64_t deltaTimeNs);
        float GetJointPosition(const AZ::Component* hingeComponent);
        float ComputeFFJointVelocity(const float currentPosition, const float desiredPosition, const rclcpp::Duration & duration) const;
        float ComputePIDJointVelocity(const float currentPosition, const float desiredPosition, const uint64_t & deltaTimeNs, int & jointIndex);
        void SetJointVelocity(AZ::Component * hingeComponent, const float desiredVelocity);

        FollowJointTrajectoryActionServer m_actionServerClass;
        AZStd::string m_ROS2ControllerName;
        bool m_initialized{false};
        bool m_initializedTrajectory{false};
        Controller m_controllerType = Controller::FeedForward;
        bool m_keepStillPositionInitialize{false};
        AZStd::vector<Controllers::PidConfiguration> m_pidConfigurationVector;
        AZStd::unordered_map<AZ::Name, AZ::EntityId> m_hierarchyMap;
        AZStd::unordered_map<AZ::Name, float> m_jointKeepStillPosition;
        trajectory_msgs::msg::JointTrajectory m_trajectory;
        rclcpp::Time m_timeStartingExecutionTraj;

        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    };
} // namespace ROS2
