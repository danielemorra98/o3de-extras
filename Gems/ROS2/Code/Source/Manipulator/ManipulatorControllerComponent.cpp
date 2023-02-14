#include <ROS2/Manipulator/ManipulatorControllerComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/functional.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/HingeJointComponent.h>
#include <ROS2/VehicleDynamics/DriveModels/PidConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>


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
            [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
            [[maybe_unused]] std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        // Dummy implementation
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal received");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::goal_cancelled_callback(
            [[maybe_unused]] const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::goal_accepted_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal accepted");
        this->m_goalHandle = goal_handle;
        m_goalStatus = GoalStatus::Active;     
    }


    // ManipulatorControllerComponent class
    void ManipulatorControllerComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        InitializePid();        
    }

    void ManipulatorControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_actionServerClass.m_actionServer.reset();
    }


    void ManipulatorControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JointPublisher"));
    }


    void ManipulatorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorControllerComponent, AZ::Component>()
                ->Version(0)
                ->Field("Controller type", &ManipulatorControllerComponent::m_controllerType)
                ->Field("PID Configuration Vector", &ManipulatorControllerComponent::m_pidConfigurationVector);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorControllerComponent>("ManipulatorControllerComponent", "[Controller for a robotic arm (only hinge joints)]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ManipulatorControllerComponent::m_controllerType,
                        "Controller type", 
                        "Different controller types to command the joints of the manipulator")
                    ->EnumAttribute(ManipulatorControllerComponent::Controller::FeedForward, "FeedForward")
                    ->EnumAttribute(ManipulatorControllerComponent::Controller::PID, "PID")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &ManipulatorControllerComponent::m_pidConfigurationVector,
                        "PIDs Configuration", 
                        "PID controllers configuration (for all the joints)");
            }
        }
    }

    void ManipulatorControllerComponent::InitializeMap()
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
                m_hierarchyMap[frameComponent->GetJointName()] = descendantID;
            }
        }
    }

    void ManipulatorControllerComponent::InitializePid()
    {
        for (auto& pid : m_pidConfigurationVector)
        {
            pid.InitializePid();
        }
    }

    void ManipulatorControllerComponent::InitializeCurrentPosition()
    {
        for (auto & [jointName , jointEntityId] : m_hierarchyMap)
        {
            AZ::Entity* jointEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(jointEntity, &AZ::ComponentApplicationRequests::FindEntity, jointEntityId);
            AZ_Assert(jointEntity, "Unknown entity %s", jointEntityId.ToString().c_str());
            if (auto* hingeComponent = jointEntity->FindComponent<PhysX::HingeJointComponent>())
            {
                m_jointKeepStillPosition[jointName] = GetJointPosition(hingeComponent);
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

    float ManipulatorControllerComponent::ComputeFFJointVelocity(const float currentPosition, const float desiredPosition, const rclcpp::Duration & duration) const
    {
        // FeedForward (dummy) method
        float desiredVelocity = (desiredPosition - currentPosition) / duration.seconds();
        return desiredVelocity;
    }

    float ManipulatorControllerComponent::ComputePIDJointVelocity(const float currentPosition, const float desiredPosition, const uint64_t & deltaTimeNs, int & jointIndex)
    {
        // PID method
        float error = desiredPosition - currentPosition;
        float command = m_pidConfigurationVector.at(jointIndex).ComputeCommand(error, deltaTimeNs);
        return command;
    }

    void ManipulatorControllerComponent::SetJointVelocity(AZ::Component * hingeComponent, const float desiredVelocity)
    {
        auto componentId = hingeComponent->GetId();
        auto entityId = hingeComponent->GetEntityId();
        const AZ::EntityComponentIdPair id(entityId,componentId);
        PhysX::JointRequestBus::Event(id, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

    void ManipulatorControllerComponent::KeepStillPosition([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        if (!m_keepStillPositionInitialize)
        {
            InitializeCurrentPosition();
            m_keepStillPositionInitialize = true;
        }
        
        int jointIndex = 0;
        for (auto & [jointName , desiredPosition] : m_jointKeepStillPosition)
        {
            float currentPosition;

            AZ::EntityId jointEntityId = m_hierarchyMap[jointName];
            AZ::Entity* jointEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(jointEntity, &AZ::ComponentApplicationRequests::FindEntity, jointEntityId);
            AZ_Assert(jointEntity, "Unknown entity %s", jointEntityId.ToString().c_str());
            if (auto* hingeComponent = jointEntity->FindComponent<PhysX::HingeJointComponent>())
            {
                currentPosition = GetJointPosition(hingeComponent);
                float desiredVelocity;
                if (m_controllerType == Controller::FeedForward)
                {
                    desiredVelocity = ComputeFFJointVelocity(
                            currentPosition, 
                            desiredPosition, 
                            rclcpp::Duration::from_nanoseconds(5e8)); // Dummy forward time reference 
                }
                else if(m_controllerType == Controller::PID)
                {
                    desiredVelocity = ComputePIDJointVelocity(
                            currentPosition, 
                            desiredPosition, 
                            deltaTimeNs,
                            jointIndex);
                }
                else
                {
                    desiredVelocity = 0.0f;
                }
                                
                SetJointVelocity(hingeComponent, desiredVelocity);
            }

            jointIndex++;
        }
    }

    void ManipulatorControllerComponent::ExecuteTrajectory([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        // If the trajectory is thoroughly executed set the status to Concluded
        if (m_trajectory.points.size() == 0)
        {
            m_initializedTrajectory = false;
            m_actionServerClass.m_goalStatus = GoalStatus::Concluded;
            return;
        }

        auto desiredGoal = m_trajectory.points.front();

        rclcpp::Duration timeFromStart = rclcpp::Duration(desiredGoal.time_from_start); // the arrival time of the current desired trajectory point
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);
        rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); // the current simulation time

        // Jump to the next point if current simulation time is ahead of timeFromStart
        if(m_timeStartingExecutionTraj + timeFromStart  <= timeNow + threshold)
        {
            m_trajectory.points.erase(m_trajectory.points.begin());
            ExecuteTrajectory(deltaTimeNs);
            return;
        }

        int jointIndex = 0;
        for (auto & jointName : m_trajectory.joint_names)
        {
            // Get the EntityId related to that joint from the hierarchy map
            // TODO: check on the existance of the name in the map
            AZ::EntityId jointEntityId = m_hierarchyMap[AZ::Name(jointName.c_str())];
            AZ::Entity* jointEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(jointEntity, &AZ::ComponentApplicationRequests::FindEntity, jointEntityId);
            AZ_Assert(jointEntity, "Unknown entity %s", jointEntityId.ToString().c_str());
            if (auto* hingeComponent = jointEntity->FindComponent<PhysX::HingeJointComponent>())
            {
                float currentPosition = GetJointPosition(hingeComponent);
                float desiredPosition = desiredGoal.positions[jointIndex];
                float desiredVelocity;
                if (m_controllerType == Controller::FeedForward)
                {
                    desiredVelocity = ComputeFFJointVelocity(
                            currentPosition, 
                            desiredPosition, 
                            m_timeStartingExecutionTraj + timeFromStart - timeNow);
                }
                else if (m_controllerType == Controller::PID)
                {
                    desiredVelocity = ComputePIDJointVelocity(
                            currentPosition, 
                            desiredPosition, 
                            deltaTimeNs,
                            jointIndex);
                }
                else
                {
                    desiredVelocity = 0.0f;
                }
                
                SetJointVelocity(hingeComponent, desiredVelocity);
            }

            jointIndex++;
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

        if (m_actionServerClass.m_goalStatus == GoalStatus::Active) // Execute the trajectory
        {
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
                m_keepStillPositionInitialize = false;
            }
        }
        else // Remain in the same position
        {
            KeepStillPosition(deltaTimeNs);
        }

    }

} // namespace ROS2
