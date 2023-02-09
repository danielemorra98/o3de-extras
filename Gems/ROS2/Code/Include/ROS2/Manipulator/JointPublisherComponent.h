#pragma once

#include <AzCore/Component/Component.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{
    //! A component responsible for publishing the 
    //! joint positions on ROS2 /joint_states topic
    //!< @see <a href="http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html">jointState message</a>.
    class JointPublisherComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JointPublisherComponent, "{a679c2e4-a602-46de-8db4-4b33d83317f4}", AZ::Component);
        JointPublisherComponent() = default;

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AZ::Component interface implementation
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        void PublishMessage();
        void UpdateMessage();
        void InitializeMap();

        float GetJointPosition(const AZ::Component* hingeComponent) const;
        void InitializeJointStateMessage();

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointstatePublisher;
        AZStd::unordered_map<AZ::Name, AZ::EntityId> m_hierarchyMap;
        sensor_msgs::msg::JointState m_jointstateMsg;
        bool m_initialized{false};
    };
} // namespace ROS2
