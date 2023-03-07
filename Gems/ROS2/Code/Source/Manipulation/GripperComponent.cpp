#include <ROS2/Manipulation/GripperComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    // GripperComponent class
    void GripperComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_ROS2gripperService = ros2Node->create_service<std_srvs::srv::SetBool>("activate_gripper", &GripperComponent::SetGripperState);

        // TODO
        // if (m_sensorConfiguration.m_visualise)
        // {
        //     auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        //     m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        // }
        // TODO
        m_rayRotations = GetGripperRayRotations();
    }

    void GripperComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void GripperComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void GripperComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GripperComponent, AZ::Component>()
                ->Version(0)
                ->Field("ROS2 Server", &GripperComponent::m_ROS2gripperService)
                ->Field("Gripper type", &GripperComponent::m_gripperType)
                ->Field("Gripper application range", &GripperComponent::m_range);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GripperComponent>("GripperComponent", "[Command the activation of the Gripper force]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_range, 
                        "Gripper range", 
                        "Maximum range at which the gripper exerts a force")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &GripperComponent::m_gripperType,
                        "Gripper type", 
                        "Different gripper designs and methods to grasp objects")
                    ->EnumAttribute(GripperComponent::Gripper::Vacuum, "Vacuum");
            }
        }
    }

    // TODO (maybe): populate with custom values
    AZStd::vector<AZ::Vector3> GripperComponent::GetGripperRayRotations()
    {
        const float minVertAngle = AZ::DegToRad(-90.0);
        const float maxVertAngle = AZ::DegToRad(90.0);
        const float minHorAngle = AZ::DegToRad(-180.0);
        const float maxHorAngle = AZ::DegToRad(180.0);

        const int verticalLayers = 32;
        const int horizontalLayers = 32;

        const float verticalStep = (maxVertAngle - minVertAngle) / static_cast<float>(verticalLayers);
        const float horizontalStep = (maxHorAngle - minHorAngle) / static_cast<float>(horizontalLayers);

        AZStd::vector<AZ::Vector3> rotations;
        for (int horLayer = 0; horLayer < horizontalLayers; horLayer++)
        {
            for (int verLayer = 0; verLayer < verticalLayers; verLayer++)
            {
                const float pitch = minVertAngle + verLayer * verticalStep;
                const float yaw = minHorAngle + horLayer * horizontalStep;

                rotations.emplace_back(AZ::Vector3(0.0f, pitch, yaw));
            }
        }

        return rotations;
    }

    AZStd::vector<AZ::Vector3> GripperComponent::RotationsToDirections(
        const AZStd::vector<AZ::Vector3>& rotations, const AZ::Vector3& rootRotation)
    {
        AZStd::vector<AZ::Vector3> directions;
        directions.reserve(rotations.size());
        for (const auto& angle : rotations)
        {
            const auto rotation = AZ::Quaternion::CreateFromEulerRadiansZYX(
                { 0.0f, -(angle.GetY() + rootRotation.GetY()), angle.GetZ() + rootRotation.GetZ() });

            directions.emplace_back(rotation.TransformVector(AZ::Vector3::CreateAxisX()));
        }

        return directions;
    }

    // TODO
    AZStd::vector<AZStd::pair<AZ::EntityId,float>> GripperComponent::PerformRaycast()
    {
        // AZStd::vector<AZ::Vector3> m_rayRotations = GetGripperRayRotations();
        AZ_Assert(m_range > 0.0f, "Ray range is not configured. Unable to Perform a raycast.");

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(m_sceneEntityId);
        }

        const AZ::Quaternion gripperTransform = GetEntity()->GetTransform()->GetWorldTM();
        const AZStd::vector<AZ::Vector3> rayDirections =
            RotationsToDirections(m_rayRotations, gripperTransform.GetEulerRadians());

        const AZ::Vector3 gripperPosition = gripperTransform.GetTranslation();

        AZStd::vector<AZStd::pair<AZ::EntityId,float>> results;
        AzPhysics::SceneQueryRequests requests;
        requests.reserve(rayDirections.size());
        results.reserve(rayDirections.size());
        for (const AZ::Vector3& direction : rayDirections)
        {
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = gripperPosition;
            request->m_direction = direction;
            request->m_distance = m_range;
            request->m_reportMultipleHits = false;
            request->m_filterCallback = [ignoredLayerIndex = this->m_ignoredLayerIndex, ignoreLayer = this->m_ignoreLayer](
                                            const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
            {
                if (ignoreLayer && (shape->GetCollisionLayer().GetIndex() == ignoredLayerIndex))
                {
                    return AzPhysics::SceneQuery::QueryHitType::None;
                }
                else
                {
                    return AzPhysics::SceneQuery::QueryHitType::Block;
                }
            };
            requests.emplace_back(AZStd::move(request));
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto requestResults = sceneInterface->QuerySceneBatch(m_sceneHandle, requests);
        AZ_Assert(requestResults.size() == rayDirections.size(), "Request size should be equal to directions size");
        for (int i = 0; i < requestResults.size(); i++)
        {
            const auto& requestResult = requestResults[i];
            if (!requestResult.m_hits.empty())
            {
                // TODO: if visualise:
                // m_raycastPoint.push_back(requestResult.m_hits[0].m_position);
                results.push_back(AZStd::make_pair(requestResult.m_hits[0].m_entityId, requestResult.m_hits[0].m_distance));
            }
        }
        return results;
    }


    // AZ::Vector3 GripperComponent::ComputeForce(const AZ::Vector3 gripperDistance)
    // {

    // }

    void GripperComponent::SetGripperState(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        m_gripperState = request->data;
        response->success = true;
    }

    void GripperComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_gripperState)
        {
            AZStd::vector<AZStd::pair<AZ::EntityId,float>> objectsDetected = PerformRaycast();
            if (!objectsDetected.empty())
            {
                for (auto& object : objectsDetected)
                {
                    AZ::EntityId objectEntityId = object.first;
                    float objectDistance = object.second;
                    AZ_TracePrintf("GripperComponent", "Find an object at %f meters", objectDistance);
                    // AZ::Vector3 gripperForce = ComputeForce(objectDistance);

                    // Physics::RigidBodyRequestBus::Event(
                    //     objectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, gripperForce * deltaTime);
                }
            }
            else
            {
                AZ_TracePrintf("GripperComponent", "Do not find any object in the nearby!");
            }
            
        }
        
    }
} // namespace ROS2