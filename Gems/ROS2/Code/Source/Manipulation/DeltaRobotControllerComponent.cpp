#include <ROS2/Manipulation/DeltaRobotControllerComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/functional.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>


namespace ROS2
{
    void DeltaRobotControllerComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        if (!m_targetControlSubscription)
        {
            auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
            AZStd::string namespacedTopic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), "desired_joint_state");

            auto ros2Node = ROS2Interface::Get()->GetNode();
            m_targetControlSubscription = ros2Node->create_subscription<sensor_msgs::msg::JointState>(
                namespacedTopic.data(),
                10,
                AZStd::bind(&DeltaRobotControllerComponent::UpdateTargetState, this, AZStd::placeholders::_1));
        }
        InitializePid();        
    }

    void DeltaRobotControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_targetControlSubscription.reset();
    }


    void DeltaRobotControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JointPublisherService"));
    }


    void DeltaRobotControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        MotorConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<DeltaRobotControllerComponent, AZ::Component>()
                ->Version(0)
                ->Field("Motors Configuration", &DeltaRobotControllerComponent::m_motors);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<DeltaRobotControllerComponent>("DeltaRobotControllerComponent", "[Controller for a (dummy) Delta Robot]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &DeltaRobotControllerComponent::m_motors,
                        "Motors Configuration", 
                        "Motors Configuration for the actuated joints of Delta Robot");
            }
        }
    }

    void DeltaRobotControllerComponent::InitializePid()
    {
        for (int i = 0; i<m_motors.size(); i++)
        {
            m_motors[i].m_motorPID.InitializePid();
        }
    }

    void DeltaRobotControllerComponent::InitializeCurrentPosition()
    {
        AZStd::array<AZ::Entity*, 3> motorEntities = {nullptr, nullptr, nullptr};
        
        for (int i = 0; i<motorEntities.size(); i++)
        {
            AZ::ComponentApplicationBus::BroadcastResult(motorEntities[i], &AZ::ComponentApplicationRequests::FindEntity, m_motors[i].m_motorEntityId);
            AZ_Assert(motorEntities[i], "Unknown entity %s", motorEntities[i]->GetId().ToString().c_str());
            auto* frameComponent = motorEntities[i]->FindComponent<ROS2FrameComponent>();
            auto* hingeComponent = motorEntities[i]->FindComponent<PhysX::HingeJointComponent>();
            if (frameComponent && hingeComponent)
            {
                m_desiredJointStateMsg.name.push_back(frameComponent->GetNamespacedJointName().GetCStr());
                m_desiredJointStateMsg.position.push_back(GetJointPosition(hingeComponent));
            }
        }
    }

    void DeltaRobotControllerComponent::UpdateTargetState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        m_desiredJointStateMsg = *msg;
    }

    float DeltaRobotControllerComponent::GetJointPosition(const AZ::Component* hingeComponent)
    {
        float position{0};
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        return position;
    }

    float DeltaRobotControllerComponent::ComputePIDJointVelocity(const float currentPosition, const float desiredPosition, const uint64_t & deltaTimeNs, int & motorIndex)
    {
        // PID method
        float error = desiredPosition - currentPosition;
        float command = m_motors[motorIndex].m_motorPID.ComputeCommand(error, deltaTimeNs);
        return command;
    }

    void DeltaRobotControllerComponent::SetJointVelocity(AZ::Component * hingeComponent, const float desiredVelocity)
    {
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::Event(id, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

    void DeltaRobotControllerComponent::GoToTarget([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        for (int motorIndex = 0; motorIndex<m_motors.size(); motorIndex++)
        {
            AZ::Entity* motorEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(motorEntity, &AZ::ComponentApplicationRequests::FindEntity, m_motors[motorIndex].m_motorEntityId);
            AZ_Assert(motorEntity, "Unknown entity %s", m_motors[motorIndex].m_motorEntityId.ToString().c_str());
            auto motorName = motorEntity->FindComponent<ROS2FrameComponent>()->GetNamespacedJointName().GetCStr();
            int msg_index = 0;
            for (auto & jointName : m_desiredJointStateMsg.name)
            {
                if (motorName == jointName)
                {
                    if (auto* hingeComponent = motorEntity->FindComponent<PhysX::HingeJointComponent>())
                    {
                        float currentPosition = GetJointPosition(hingeComponent);
                        float desiredPosition = m_desiredJointStateMsg.position[msg_index];
                        float desiredVelocity = ComputePIDJointVelocity(
                            currentPosition, 
                            desiredPosition, 
                            deltaTimeNs,
                            motorIndex);
                        
                        SetJointVelocity(hingeComponent, desiredVelocity);
                    }
                }
                msg_index++;
            }
        }  
    }

    void DeltaRobotControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            InitializeCurrentPosition();
            m_initialized = true;
        }

        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        GoToTarget(deltaTimeNs);
    }

} // namespace ROS2
