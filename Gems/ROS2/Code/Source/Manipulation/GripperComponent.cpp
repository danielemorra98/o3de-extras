#include <ROS2/Manipulation/GripperComponent.h>
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/Shape.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    // GripperComponent class
    void GripperComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_ROS2gripperService = ros2Node->create_service<std_srvs::srv::SetBool>("activate_gripper", AZStd::bind(&GripperComponent::SetGripperState, this, AZStd::placeholders::_1, AZStd::placeholders::_2));

        if (m_visualise)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }
        
        m_rayRelativeRotations = GetGripperRayRotations();
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
                // ->Field("ROS2 Server", &GripperComponent::m_ROS2gripperService)
                ->Field("Gripper type", &GripperComponent::m_gripperType)
                ->Field("Visualization toggle", &GripperComponent::m_visualise)
                ->Field("Vertical Layers", &GripperComponent::m_verticalLayers)
                ->Field("Horizontal Layers", &GripperComponent::m_horizontalLayers)
                ->Field("Vertical Range", &GripperComponent::m_rangeVertical)
                ->Field("Max Gripper Force", &GripperComponent::m_maxGripperForce)
                ->Field("Horizontal Range", &GripperComponent::m_rangeHorizontal)
                ->Field("Ignore Layer Collision", &GripperComponent::m_ignoreLayerCollision)
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
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_visualise, 
                        "Visualization toggle", 
                        "Debug visualization which draws the points of the raycast")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_verticalLayers, 
                        "Vertical Layers", 
                        "Number of vertical raycast layer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_horizontalLayers, 
                        "Horizontal Layers", 
                        "Number of Horizontal raycast layer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_rangeVertical, 
                        "Vertical range", 
                        "Deg of cone aperture (vertical)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_rangeHorizontal, 
                        "Horizontal range", 
                        "Deg of cone aperture (horizontal)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, 
                        &GripperComponent::m_maxGripperForce, 
                        "Max Gripper Force", 
                        "Maximum Force that the Gripper could exert")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GripperComponent::m_ignoreLayerCollision,
                        "Ignore Layer Collision", 
                        "Index of collision layer to be ignored by the gripper")
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
        const float minVertAngle = AZ::DegToRad(-m_rangeVertical);
        const float maxVertAngle = AZ::DegToRad(m_rangeVertical);
        const float minHorAngle = AZ::DegToRad(-m_rangeHorizontal);
        const float maxHorAngle = AZ::DegToRad(m_rangeHorizontal);

        const float verticalStep = (maxVertAngle - minVertAngle) / static_cast<float>(m_verticalLayers);
        const float horizontalStep = (maxHorAngle - minHorAngle) / static_cast<float>(m_horizontalLayers);

        AZStd::vector<AZ::Vector3> rotations;
        for (int horLayer = 0; horLayer < m_horizontalLayers; horLayer++)
        {
            for (int verLayer = 0; verLayer < m_verticalLayers; verLayer++)
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
            // directions.emplace_back(rotation.GetEulerRadians());
        }

        return directions;
    }

    AzPhysics::SceneHandle GripperComponent::GetPhysicsSceneFromEntityId(const AZ::EntityId& entityId)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto foundBody = physicsSystem->FindAttachedBodyHandleFromEntityId(entityId);
        AzPhysics::SceneHandle lidarPhysicsSceneHandle = foundBody.first;
        if (foundBody.first == AzPhysics::InvalidSceneHandle)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            lidarPhysicsSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        }

        AZ_Assert(lidarPhysicsSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid physics scene handle for entity");
        return lidarPhysicsSceneHandle;
    }

    AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> GripperComponent::PerformRaycast()
    {
        AZ_Assert(!m_rayRelativeRotations.empty(), "Ray poses are not configured. Unable to Perform a raycast.");
        AZ_Assert(m_range > 0.0f, "Ray range is not configured. Unable to Perform a raycast.");

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(GetEntityId());
        }

        const AZ::Transform gripperTransform = GetEntity()->GetTransform()->GetWorldTM();
        const AZStd::vector<AZ::Vector3> rayAbsoluteDirections =
            RotationsToDirections(m_rayRelativeRotations, gripperTransform.GetEulerRadians());

        const AZ::Vector3 gripperPosition = gripperTransform.GetTranslation();

        AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> results;
        AzPhysics::SceneQueryRequests requests;
        requests.reserve(rayAbsoluteDirections.size());
        results.reserve(rayAbsoluteDirections.size());
        for (const AZ::Vector3& direction : rayAbsoluteDirections)
        {
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = gripperPosition;
            request->m_direction = direction;
            request->m_distance = m_range;
            request->m_reportMultipleHits = false;
            request->m_filterCallback = [ignoredLayerIndex = m_ignoreLayerCollision](
                                            const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
            {
                if (shape->GetCollisionLayer().GetIndex() == ignoredLayerIndex)
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
        AZ_Assert(requestResults.size() == rayAbsoluteDirections.size(), "Request size should be equal to directions size");
        for (int i = 0; i < requestResults.size(); i++)
        {
            const auto& requestResult = requestResults[i];
            if (!requestResult.m_hits.empty())
            {
                // TODO: 
                if (m_visualise)
                {
                    m_raycastPoint.push_back(requestResult.m_hits[0].m_position);
                }

                // AZ::Vector3 gripperForce = ComputeForce(requestResult.m_hits[0].m_distance, rayAbsoluteDirections.at(rayAbsoluteDirections.size()-i-1));
                AZ::Vector3 gripperForce = ComputeForce(requestResult.m_hits[0].m_distance, rayAbsoluteDirections.at(i));
                results.push_back(AZStd::make_pair(requestResult.m_hits[0].m_entityId, gripperForce));
            }
        }
        return results;
    }


    AZ::Vector3 GripperComponent::ComputeForce(const float gripperDistance, const AZ::Vector3 normalizeDirection)
    {
        AZ_Assert(m_maxGripperForce > 0.0f, "Gripper maximum force is not configured. Unable to Apply Force on close objects.");
        float arctan_model_gripper = - 2 / 3.1416 * AZStd::atan(10*gripperDistance/m_range) + 1;
        float force_magnitude = m_maxGripperForce * arctan_model_gripper;
        AZ::Vector3 force = - force_magnitude * normalizeDirection;
        return force;
    }

    void GripperComponent::SetGripperState(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        m_gripperState = request->data;
        response->success = true;
    }

    void GripperComponent::Visualise()
    {
        if (m_drawQueue)
        {
            const uint8_t pixelSize = 5;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = m_raycastPoint.data();
            drawArgs.m_vertCount = m_raycastPoint.size();
            drawArgs.m_colors = &AZ::Colors::Red;
            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawPoints(drawArgs);
        }
    }

    void GripperComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_gripperState)
        {
            AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> objectsDetected = PerformRaycast();
            if (!objectsDetected.empty())
            {
                for (auto& object : objectsDetected)
                {
                    AZ::EntityId objectEntityId = object.first;
                    AZ::Vector3 gripperForce = object.second;
                    // AZ_TracePrintf("GripperComponent", "Find the object %s at %f meters", objectEntityId.ToString().c_str(), objectDistance);

                    Physics::RigidBodyRequestBus::Event(objectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, gripperForce * deltaTime);
                }
            }
            else
            {
                AZ_TracePrintf("GripperComponent", "Do not find any object in the nearby!");
            }

            if (m_visualise)
            {
                Visualise();
                m_raycastPoint.clear();
            }
            
        }
        
    }
} // namespace ROS2