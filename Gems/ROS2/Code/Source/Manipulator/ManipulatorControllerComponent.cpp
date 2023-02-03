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
        this->m_actionServer = rclcpp_action::create_server<FollowJointTrajectory>(
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
        AZ_Printf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal received");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::goal_cancelled_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        AZ_Printf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal canceled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::goal_accepted_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        AZ_Printf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal accepted");
        this->m_goalHandle = goal_handle;
        m_goalStatus = GoalStatus::Active;     
    }

    // std::shared_ptr<GoalHandleFollowJointTrajectory> FollowJointTrajectoryActionServer::GetGoal()
    // {

    // }


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

    void ManipulatorControllerComponent::DebugTrajectoryExecution()
    {
        // Dummy-debug implementation
        if(!m_debugBool)
        {
            std::shared_ptr<const FollowJointTrajectory::Goal> goal = m_actionServerClass.m_goalHandle->get_goal();
            AZ_Printf("ManipulatorControllerComponent", "Executing manipulator goal");
            AZ_Printf("ManipulatorControllerComponent", "First Trajectory Point: %f, %f, %f, %f, %f, %f, %f",
                    goal->trajectory.points[0].positions[0],
                    goal->trajectory.points[0].positions[1],
                    goal->trajectory.points[0].positions[2],
                    goal->trajectory.points[0].positions[3],
                    goal->trajectory.points[0].positions[4],
                    goal->trajectory.points[0].positions[5],
                    goal->trajectory.points[0].positions[6]);
                
            m_debugBool = true;
        }
    }

    void ManipulatorControllerComponent::ExecuteTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
    {
        if (m_trajectory.points.size() == 0)
        {
            m_trajectory = trajectory;
        }

        auto desired_goal = m_trajectory.points.front();
        m_trajectory.points.erase(m_trajectory.points.begin());

        // First point is the current state of the robot --> jump to the second point
        if(desired_goal.time_from_start. < rclcpp::Duration(1e7)) // needs to be reviewed
        {
            ExecuteTrajectory(trajectory);
            return;
        }

        // ComputeJointVelocity(desired_goal);

        // SetVelocityJoints();

        // If the trajectory is thoroughly executed set the status to Concluded
        if (m_trajectory.points.size() == 0)
        {
            m_actionServerClass.m_goalStatus = GoalStatus::Concluded;
        }
        
    }

    void ManipulatorControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            InitializeMap();
            m_initialized = true;
        }

        AZ_Printf("ManipulatorControllerComponent", "Time: %fs and %fms", time.GetSeconds(), time.GetMilliseconds());

        // // Goal Execution
        // if (m_actionServerClass.m_goalStatus == GoalStatus::Active)
        // {
        //     // DebugTrajectoryExecution()

        //     m_actionServerClass.m_goalHandle->get_goal()->trajectory;
        //     ExecuteTrajectory(m_actionServerClass.m_goalHandle->get_goal()->trajectory);
        // }

        // if (m_goalStatus == GoalStatus::Concluded)
        // {
        //     m_goalStatus == GoalStatus::Pending;
        //     auto result = std::make_shared<FollowJointTrajectory::Result>();
        //     m_actionServerClass.m_goalHandle->succeed(result);
        // }
    }

} // namespace ROS2
