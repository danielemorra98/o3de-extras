#include <ROS2/Manipulation/JointPublisherComponent.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include <ROS2/ROS2Bus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
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


    void JointPublisherComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointPublisherComponent, AZ::Component>()
                ->Version(0)
                ->Field("Frequency (HZ)", &JointPublisherComponent::m_frequency);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointPublisherComponent>("JointPublisherComponent", "[Publish all the Hinge joint in the tree]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &JointPublisherComponent::m_frequency, "Frequency", "Frequency of publishing [Hz]");
            }
        }
    }

    void JointPublisherComponent::InitializeMap()
    {
        AZStd::vector<AZ::EntityId> descendants;
        AZ::TransformBus::EventResult(descendants, GetEntityId(), &AZ::TransformInterface::GetAllDescendants);

        for (const AZ::EntityId& descendantID : descendants)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, descendantID);
            AZ_Assert(entity, "Unknown entity %s", descendantID.ToString().c_str());
            auto* frameComponent = entity->FindComponent<ROS2FrameComponent>();
            auto* hingeComponent = entity->FindComponent<PhysX::HingeJointComponent>();
            if (frameComponent && hingeComponent)
            {
                m_hierarchyMap[frameComponent->GetNamespacedJointName()] = descendantID;
            }
        }
    }

    AZStd::unordered_map<AZ::Name, AZ::EntityId>  &JointPublisherComponent::GetHierarchyMap()
    {
        return m_hierarchyMap;
    }

    void JointPublisherComponent::InitializeJointStateMessage()
    {
        InitializeMap();
        for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
        {
            AZ::Entity* hingeEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(hingeEntity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            AZ_Assert(hingeEntity, "Unknown entity %s", entityId.ToString().c_str());
            if (hingeEntity && hingeEntity->FindComponent<PhysX::HingeJointComponent>() != nullptr)
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

        auto frameTime = m_frequency == 0 ? 1 : 1 / m_frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }

        // Note that the publisher frequency can be limited by simulation tick rate (if higher frequency is desired).
        PublishMessage();
    }
} // namespace ROS2
