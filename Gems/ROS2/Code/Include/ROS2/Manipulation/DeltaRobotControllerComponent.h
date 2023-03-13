#pragma once

#include <AzCore/Component/Component.h>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Manipulation/MotorConfiguration.h>


namespace ROS2
{
    class DeltaRobotControllerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(DeltaRobotControllerComponent, "{2900a1c0-7c99-469a-af04-fe0182f765e9}", AZ::Component);

        DeltaRobotControllerComponent() = default;
        ~DeltaRobotControllerComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void InitializePid();
        void InitializePositionMsg();
        void InitializeMotorConfiguration();
        void UpdateTargetState(const sensor_msgs::msg::JointState::SharedPtr msg);
        // void KeepStillPosition(const uint64_t deltaTimeNs);
        void GoToTarget(const uint64_t deltaTimeNs);
        float GetJointPosition(const AZ::Component* hingeComponent);
        float ComputePIDJointVelocity(const float currentPosition, const float desiredPosition, const uint64_t & deltaTimeNs, int & motorIndex);
        void SetJointVelocity(AZ::Component * hingeComponent, const float desiredVelocity);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_targetControlSubscription;
        bool m_initialized{false};
        // bool m_keepStillPositionInitialize{false};
        AZStd::array<MotorConfiguration, 3> m_motors;
        AZStd::unordered_map<AZ::Name, AZ::EntityId> m_hierarchyMap;
        sensor_msgs::msg::JointState m_desiredJointStateMsg;

    };
} // namespace ROS2
