#include <ROS2/Manipulator/JointPublisherComponent.h>
#include <AzCore/Serialization/EditContext.h>
#include "ROS2/ROS2Bus.h"
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

namespace ROS2
{
    void JointPublisherComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        // auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        // // const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), "joint_states");
        // m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);        // TODO: add QoS instead of "1"
        // InitializeJointStateMessage();
    }

    void JointPublisherComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void JointPublisherComponent::Reflect(AZ::ReflectContext* context)
    {
        // if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        // {
        //     serialize->Class<JointPublisherComponent, AZ::Component>()->Version(0);

        //     if (AZ::EditContext* ec = serialize->GetEditContext())
        //     {
        //         ec->Class<JointPublisherComponent>("JointPublisherComponent", "[Publish all the Hinge joint in the tree]")
        //             ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
        //             ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
        //             ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
        //     }
        // }
    }

    void JointPublisherComponent::InitializeJointStateMessage()
    {
        // double nullPosition{0};
        // for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
        // {
        //     m_jointstate_msg.name.push_back(name.GetCStr());
        //     m_jointstate_msg.position.push_back(nullPosition);
        // }
    }

    void JointPublisherComponent::UpdateMessage()
    {
        // int i = 0;
        // for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
        // {
        //     m_jointstate_msg.position[i] = GetJointPosition(AzToolsFramework::GetEntityById(entityId));
        //     i++;
        // }
    }

    double JointPublisherComponent::GetJointPosition(const AZ::Entity* hingeEntity) const
    {
        // double position{0};
        // auto* hingeComponent = hingeEntity->FindComponent<PhysX::HingeJointComponent>();
        // auto componentId = hingeComponent->GetId();
        // const AZ::EntityComponentIdPair id(hingeEntity->GetId(),componentId);
        // PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        // return position;
        return 0;
    }

    void JointPublisherComponent::PublishMessage()
    {
        // std_msgs::msg::Header ros_header;
        // ros_header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        // m_jointstate_msg.header = ros_header;
        // UpdateMessage();
        // m_jointstatePublisher->publish(m_jointstate_msg);
    }


    void JointPublisherComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // if(!m_initialized)
        // {
        //     InitializeJointStateMessage();
        //     m_initialized = true;
        // }

        // PublishMessage();
    }
} // namespace ROS2
