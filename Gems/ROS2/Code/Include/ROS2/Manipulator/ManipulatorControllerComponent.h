#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace ROS2
{
    class FollowJointTrajectoryActionServer
    {
    public:
        using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        FollowJointTrajectoryActionServer();

        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

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

        AZ_COMPONENT(ManipulatorControllerComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component); // , ManipulatorRequestBus::Handler);

        // AZ::Component interface implementation.
        ManipulatorControllerComponent() = default;
        ~ManipulatorControllerComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void InitializeMap();

        FollowJointTrajectoryActionServer m_actionServerClass;
        bool m_initialized{false};
        AZStd::unordered_map<AZ::Name, AZ::EntityId> m_hierarchyMap;

    };
} // namespace ROS2
