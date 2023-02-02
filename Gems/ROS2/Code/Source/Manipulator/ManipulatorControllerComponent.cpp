#include <ROS2/Manipulator/ManipulatorControllerComponent.h>
#include <RobotImporter/URDFMetadataComponent.h>
#include "ROS2/ROS2Bus.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/std/functional.h>


// #include ROS_things


namespace ROS2
{
    // ROS2 Action Server class
    FollowJointTrajectoryActionServer::FollowJointTrajectoryActionServer()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        // Create the ROS2 action server
        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            ros2Node,
            "panda_arm_controller/follow_joint_trajectory",
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_received_callback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_cancelled_callback, this, AZStd::placeholders::_1),
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_accepted_callback, this, AZStd::placeholders::_1));

    }

    rclcpp_action::GoalResponse FollowJointTrajectoryActionServer::goal_received_callback(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        // Dummy implementation
        // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::goal_cancelled_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::goal_accepted_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        goal_handle->succeed(result);
    }


    // ManipulatorControllerComponent class
    void ManipulatorControllerComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();

    }

    void ManipulatorControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ManipulatorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorControllerComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorControllerComponent>("ManipulatorControllerComponent", "[Controller for a robotic arm (only hinge joints)]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void ManipulatorControllerComponent::InitializeMap()
    {
        auto* metadataComponent = GetEntity()->FindComponent<URDFMetadataComponent>();
        m_hierarchyMap = metadataComponent->GetHierarchy();
        AZ_Printf("ManipulatorControllerComponent", "Map initialized");
    }

    // void ManipulatorControllerComponent::InitializeJointStateMessage()
    // {
    //     InitializeMap();
    //     double nullPosition{0};
    //     for ([[maybe_unused]] auto& [name, entityId] : m_hierarchyMap)
    //     {
    //         AZ::Entity* hingeEntity = AzToolsFramework::GetEntityById(entityId);
    //         AZ_Assert(hingeEntity, "Unknown entity %s", entityId.ToString().c_str());
            
    //         if (hingeEntity->FindComponent<PhysX::HingeJointComponent>() != nullptr)
    //         {
    //             m_jointstate_msg.name.push_back(name.GetCStr());
    //             m_jointstate_msg.position.push_back(nullPosition);
    //         }
    //     }
    // }

    // double ManipulatorControllerComponent::GetJointPosition(const AZ::Component* hingeComponent) const
    // {
    //     double position{0};
    //     auto componentId = hingeComponent->GetId();
    //     auto entityId = hingeComponent->GetEntityId();
    //     const AZ::EntityComponentIdPair id(entityId,componentId);
    //     PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
    //     return position;
    // }

    void ManipulatorControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            InitializeMap();
            m_initialized = true;
        }
    }

} // namespace ROS2
