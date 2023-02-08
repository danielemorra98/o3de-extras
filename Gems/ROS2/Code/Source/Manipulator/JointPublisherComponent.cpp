#include <ROS2/Manipulator/JointPublisherComponent.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/ROS2Bus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/URDFMetadataComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void JointPublisherComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        AZStd::string namespacedTopic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), "joint_states");
        m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>(namespacedTopic.data(), rclcpp::QoS(1));        // TODO: add QoS instead of "1"
    }

    void JointPublisherComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_jointstatePublisher.reset();
    }

    void JointPublisherComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointPublisher"));
    }

    void JointPublisherComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("URDFMetadata"));
    }

    void JointPublisherComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointPublisher"));
    }

    void JointPublisherComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("URDFMetadata"));
    }

    void JointPublisherComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointPublisherComponent, AZ::Component>()
                ->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointPublisherComponent>("JointPublisherComponent", "[Publish all the Hinge joint in the tree]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void JointPublisherComponent::InitializeMap()
    {
        if (auto* metadataComponent = GetEntity()->FindComponent<URDFMetadataComponent>())
        {
            m_hierarchyMap = metadataComponent->GetHierarchy();
            AZ_TracePrintf("JointPublisherComponent", "Map initialized");
        }
    }

    void JointPublisherComponent::InitializeJointStateMessage()
    {
        InitializeMap();
        for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
        {
            AZ::Entity* hingeEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(hingeEntity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            AZ_Assert(hingeEntity, "Unknown entity %s", entityId.ToString().c_str());
            if (hingeEntity->FindComponent<PhysX::HingeJointComponent>() != nullptr)
            {
                m_jointstateMsg.name.push_back(name.GetCStr());
                m_jointstateMsg.position.push_back(0.0f);
            }
        }
    }

    void JointPublisherComponent::UpdateMessage()
    {
        int i = 0;
        for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
        {
            AZ::Entity* hingeEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(hingeEntity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            AZ_Assert(hingeEntity, "Unknown entity %s", entityId.ToString().c_str());
            if (auto* hingeComponent = hingeEntity->FindComponent<PhysX::HingeJointComponent>())
            {
                m_jointstateMsg.position[i] = GetJointPosition(hingeComponent);
            }
            i++;
        }
    }

    float JointPublisherComponent::GetJointPosition(const AZ::Component* hingeComponent) const
    {
        float position{0};
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        return position;
    }

    void JointPublisherComponent::PublishMessage()
    {
        std_msgs::msg::Header ros_header;
        ros_header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointstateMsg.header = ros_header;
        UpdateMessage();
        m_jointstatePublisher->publish(m_jointstateMsg);
    }


    void JointPublisherComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            InitializeJointStateMessage();
            m_initialized = true;
        }

        PublishMessage();
    }
} // namespace ROS2
