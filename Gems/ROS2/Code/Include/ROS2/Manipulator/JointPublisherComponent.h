#pragma once

#include <ROS2/Manipulator/MotorizedJointComponent.h>
#include <RobotImporter/URDFMetadataComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{
    //! A component responsible for storing the jointComponent tree structure
    //! and each of the joint's name as they are described in URDF
    class JointPublisherComponent
        : public URDFMetadataComponent
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JointPublisherComponent, "{a679c2e4-a602-46de-8db4-4b33d83317f4}", AZ::Component);
        JointPublisherComponent() = default;

        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        void PublishMessage();
        void UpdateMessage();
        void InitializeMap();

        double GetJointPosition(const AZ::Component* hingeComponent) const;
        void InitializeJointStateMessage();

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointstatePublisher;
        sensor_msgs::msg::JointState m_jointstateMsg;
        bool m_initialized{false};
    };
} // namespace ROS2
