#include <ROS2/Manipulator/ManipulatorControllerComponent.h>
#include <RobotImporter/URDFMetadataComponent.h>
#include "ROS2/ROS2Bus.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/std/functional.h>
#include <ROS2/VehicleDynamics/DriveModels/PidConfiguration.h>


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
        InitializePid();        
    }

    void ManipulatorControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ManipulatorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        // VehicleDynamics::PidConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorControllerComponent, AZ::Component>()
                ->Version(0)
                ->Field("PID boolean", &ManipulatorControllerComponent::m_pidBoolean)
                ->Field("PID Configuration Vector", &ManipulatorControllerComponent::m_pidConfigurationVector);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorControllerComponent>("ManipulatorControllerComponent", "[Controller for a robotic arm (only hinge joints)]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &ManipulatorControllerComponent::m_pidBoolean,
                        "Using PID", 
                        "Boolean value for the choice of having a PID instead of a feedforward controller")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &ManipulatorControllerComponent::m_pidConfigurationVector,
                        "PID Configuration", 
                        "PID controller configuration valid for all the hinge joints");
                    // ->Attribute(AZ::Edit::Attributes::Visibility, &ManipulatorControllerComponent::m_pidBoolean);
            }
        }
    }

    void ManipulatorControllerComponent::InitializeMap()
    {
        auto* metadataComponent = GetEntity()->FindComponent<URDFMetadataComponent>();
        m_hierarchyMap = metadataComponent->GetHierarchy();
        AZ_Printf("ManipulatorControllerComponent", "Map initialized");
    }

    void ManipulatorControllerComponent::InitializePid()
    {
        for (auto& pid : m_pidConfigurationVector)
        {
            pid.InitializePid();
            // AZ_Printf("ManipulatorControllerComponent", "Initialization n. %d PID", i); // DEBUG
        }
    }

    void ManipulatorControllerComponent::InitializeCurrentPosition()
    {
        for (auto [joint_name , joint_entity_id] : m_hierarchyMap)
        {
            if (auto* hingeComponent = AzToolsFramework::GetEntityById(joint_entity_id)->FindComponent<PhysX::HingeJointComponent>())
            {
                m_jointKeepStillPosition[joint_name] = GetJointPosition(hingeComponent);
            }
        }
    }

    float ManipulatorControllerComponent::GetJointPosition(const AZ::Component* hingeComponent)
    {
        float position{0};
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::EventResult(position, id, &PhysX::JointRequests::GetPosition);
        return position;
    }

    double ManipulatorControllerComponent::ComputeFFJointVelocity(float & currentPosition, float & desiredPosition, const rclcpp::Duration & duration)
    {
        // FeedForward (dummy) method
        double desired_velocity = (desiredPosition - currentPosition) / duration.seconds();
        return desired_velocity;
    }

    double ManipulatorControllerComponent::ComputePIDJointVelocity(float & currentPosition, float & desiredPosition, const uint64_t & deltaTimeNs, int & joint_index)
    {
        // PID method
        double error = desiredPosition - currentPosition;
        // AZ_Printf("ManipulatorControllerComponent", "deltaTimeNs: %d; error PID joint n. %d: %f", deltaTimeNs, joint_index, error); // DEBUG
        double desired_velocity = m_pidConfigurationVector.at(joint_index).ComputeCommand(error, deltaTimeNs);
        return desired_velocity;
    }

    void ManipulatorControllerComponent::SetJointVelocity(AZ::Component * hingeComponent, double & desiredVelocity)
    {
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::Event(id, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

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

    void ManipulatorControllerComponent::KeepStillPosition([[maybe_unused]] const uint64_t & deltaTimeNs)
    {
        if (!m_KeepStillPositionInitialize)
        {
            InitializeCurrentPosition();
            m_KeepStillPositionInitialize = true;
        }
        
        int joint_index = 0;
        for (auto [joint_name , desired_position] : m_jointKeepStillPosition)
        {
            float current_position;
            // Get the EntityId related to that joint from the hierarchy map
            // TODO: check on the existance of the name in the map
            AZ::EntityId joint_entity_id = m_hierarchyMap[joint_name];
            if (auto* hingeComponent = AzToolsFramework::GetEntityById(joint_entity_id)->FindComponent<PhysX::HingeJointComponent>())
            {
                current_position = GetJointPosition(hingeComponent);
                double desired_velocity;
                if (!m_pidBoolean)
                {
                    desired_velocity = ComputeFFJointVelocity(
                            current_position, 
                            desired_position, 
                            rclcpp::Duration::from_nanoseconds(5e8)); // Dummy forward time reference 
                }
                else
                {
                    desired_velocity = ComputePIDJointVelocity(
                            current_position, 
                            desired_position, 
                            deltaTimeNs,
                            joint_index);
                }
                
                AZ_Printf("ManipulatorControllerComponent", "Desired velocity for joint %d: %f", joint_index, desired_velocity);
                
                SetJointVelocity(hingeComponent, desired_velocity);
            }

            joint_index++;
        }
    }

    void ManipulatorControllerComponent::ExecuteTrajectory([[maybe_unused]] const uint64_t & deltaTimeNs)
    {
        // If the trajectory is thoroughly executed set the status to Concluded
        if (m_trajectory.points.size() == 0)
        {
            m_initializedTrajectory = false;
            m_actionServerClass.m_goalStatus = GoalStatus::Concluded;
            return;
        }

        auto desired_goal = m_trajectory.points.front();

        // TODO: give a brief description
        rclcpp::Duration time_from_start = rclcpp::Duration(desired_goal.time_from_start);
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);
        rclcpp::Time time_now = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());

        // AZ_Printf("ManipulatorControllerComponent", "Desired Arrival time of trajectory point: %f",
        //             (m_timeStartingExecutionTraj + time_from_start - time_now).seconds()); // DEBUG

        if(m_timeStartingExecutionTraj + time_from_start  <= time_now + threshold) // needs to be reviewed
        {
            m_trajectory.points.erase(m_trajectory.points.begin());
            ExecuteTrajectory(deltaTimeNs);
            return;
        }

        int joint_index = 0;
        for (auto joint_name : m_trajectory.joint_names)
        {
            float current_position;
            float desired_position = desired_goal.positions[joint_index];
            // Get the EntityId related to that joint from the hierarchy map
            // TODO: check on the existance of the name in the map
            AZ::EntityId joint_entity_id = m_hierarchyMap[AZ::Name(joint_name.c_str())];
            if (auto* hingeComponent = AzToolsFramework::GetEntityById(joint_entity_id)->FindComponent<PhysX::HingeJointComponent>())
            {
                current_position = GetJointPosition(hingeComponent);
                double desired_velocity;
                if (!m_pidBoolean)
                {
                    desired_velocity = ComputeFFJointVelocity(
                            current_position, 
                            desired_position, 
                            m_timeStartingExecutionTraj + time_from_start - time_now);
                }
                else
                {
                    desired_velocity = ComputePIDJointVelocity(
                            current_position, 
                            desired_position, 
                            deltaTimeNs,
                            joint_index);
                }
                
                
                SetJointVelocity(hingeComponent, desired_velocity);
            }

            joint_index++;
        }

        
    }

    void ManipulatorControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(!m_initialized)
        {
            InitializeMap();
            m_initialized = true;
        }

        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;

        // Goal Execution
        if (m_actionServerClass.m_goalStatus == GoalStatus::Active)
        {
            // DebugTrajectoryExecution()
            // AZ_Printf("ManipulatorControllerComponent", "Executing Trajectory"); // DEBUG
            if (!m_initializedTrajectory)
            {
                m_trajectory = m_actionServerClass.m_goalHandle->get_goal()->trajectory;
                m_timeStartingExecutionTraj = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
                m_initializedTrajectory = true;
            }

            ExecuteTrajectory(deltaTimeNs);

            if (m_actionServerClass.m_goalStatus == GoalStatus::Concluded)
            {
                m_actionServerClass.m_goalStatus = GoalStatus::Pending;
                auto result = std::make_shared<FollowJointTrajectory::Result>();
                m_actionServerClass.m_goalHandle->succeed(result);
                m_KeepStillPositionInitialize = false;
            }
        }
        else
        {
            KeepStillPosition(deltaTimeNs);
        }

    }

} // namespace ROS2
