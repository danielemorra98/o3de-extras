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
                ->Field("Cone Angle", &GripperComponent::m_coneAngle)
                ->Field("Max Gripper Force", &GripperComponent::m_maxGripperForce)
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
                        &GripperComponent::m_coneAngle, 
                        "Gripper Cone angle", 
                        "Angle [deg] of the cone in which the gripper exerts forces on object(s)")
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

    AZStd::vector<AZ::Vector3> GripperComponent::GetGripperRayRotations()
    {
        AZ_Assert(m_coneAngle > 0.0f, "Gripper Cone angle not initialized");
        float minPitch = AZ::DegToRad(-m_coneAngle);
        float minYaw = AZ::DegToRad(-90.0f);

        int pitchLayers = (int)((2*m_coneAngle)/m_resolution);
        int yawLayers = (int)(180/m_resolution);

        AZStd::vector<AZ::Vector3> rotations;
        for (int stepPitch = 0; stepPitch < pitchLayers; stepPitch++)
        {
            for (int stepYaw = 0; stepYaw < yawLayers; stepYaw++)
            {
                const float pitch = minPitch + stepPitch * AZ::DegToRad(m_resolution);
                const float yaw = minYaw + stepYaw * AZ::DegToRad(m_resolution);
                rotations.emplace_back(AZ::Vector3(yaw, pitch, 0.0f));
            }
        }

        return rotations;
    }

    static AZ::Transform RotateAroundLocalHelper(float eulerAngleRadian, const AZ::Transform& TM, AZ::Vector3 axis)
    {
        //normalize the axis before creating rotation
        axis.Normalize();
        AZ::Quaternion rotate = AZ::Quaternion::CreateFromAxisAngle(axis, eulerAngleRadian);

        AZ::Transform newTM = TM;
        newTM.SetRotation((rotate * TM.GetRotation()).GetNormalized());
        return newTM;
    }

    AZStd::vector<AZ::Vector3> GripperComponent::RotationsToDirections(
        const AZStd::vector<AZ::Vector3>& rotations, const AZ::Transform& rootTransform)
    {
        AZStd::vector<AZ::Vector3> directions;
        AZ::Vector3 yAxis = rootTransform.GetBasisY();
        AZ::Vector3 xAxis = rootTransform.GetBasisX();
        directions.reserve(rotations.size());
        for (const auto& angle : rotations)
        {
            AZ::Transform step1TM = RotateAroundLocalHelper(angle.GetY(), rootTransform, yAxis);
            AZ::Transform step2TM = RotateAroundLocalHelper(angle.GetX(), step1TM, xAxis);

            directions.emplace_back(step2TM.TransformVector(AZ::Vector3::CreateAxisX()));
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

        m_mapObjectMinDistance.clear();

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(GetEntityId());
        }

        const AZ::Transform gripperTransform = GetEntity()->GetTransform()->GetWorldTM();
        const AZStd::vector<AZ::Vector3> rayAbsoluteDirections =
            RotationsToDirections(m_rayRelativeRotations, gripperTransform);

        const AZ::Vector3 gripperPosition = gripperTransform.GetTranslation();

        AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> results;
        AzPhysics::SceneQueryRequests requests;
        requests.reserve(rayAbsoluteDirections.size());
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
                if (m_visualise)
                {
                    m_raycastPoint.push_back(requestResult.m_hits[0].m_position);
                }

                if (requestResult.m_hits[0].m_distance > 0.005 && m_mapObjectMinDistance.find(requestResult.m_hits[0].m_entityId) != m_mapObjectMinDistance.end())
                {
                    AZ::Vector3 gripperForce = ComputeForce(requestResult.m_hits[0].m_distance, rayAbsoluteDirections.at(i));
                    results.push_back(AZStd::make_pair(requestResult.m_hits[0].m_entityId, gripperForce));
                }
                else
                {
                    if (m_mapObjectMinDistance.find(requestResult.m_hits[0].m_entityId) != m_mapObjectMinDistance.end())
                    {
                        if (requestResult.m_hits[0].m_distance <= m_mapObjectMinDistance[requestResult.m_hits[0].m_entityId])
                        {
                            m_mapObjectMinDistance[requestResult.m_hits[0].m_entityId] = requestResult.m_hits[0].m_distance;
                        }
                    }
                    else
                    {
                        m_mapObjectMinDistance[requestResult.m_hits[0].m_entityId] = requestResult.m_hits[0].m_distance;
                    }
                }
            }
        }
        return results;
    }


    AZ::Vector3 GripperComponent::ComputeForce(const float gripperDistance, const AZ::Vector3 normalizeDirection)
    {
        AZ_Assert(m_maxGripperForce > 0.0f, "Gripper maximum force is not configured. Unable to Apply Force on close objects.");
        float arctan_model_gripper = - 2 / 3.1416 * AZStd::atan(10*gripperDistance/m_range) + 1;
        int total_layer = (2 * m_coneAngle) * 180 / m_resolution;
        float force_magnitude = m_maxGripperForce / total_layer * arctan_model_gripper;
        AZ::Vector3 force = force_magnitude * normalizeDirection;
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
            AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> intersections = PerformRaycast();

            if (intersections.size() > 0)
            {
                AZ_TracePrintf("GripperComponent", "Apply the force to the object(s) grasped");
                for (auto& rayintersected : intersections)
                {
                    AZ::EntityId objectEntityId = rayintersected.first;
                    AZ::Vector3 gripperForce = rayintersected.second;
                    Physics::RigidBodyRequestBus::Event(objectEntityId, &Physics::RigidBodyRequests::ApplyLinearImpulse, - gripperForce * deltaTime);
                    Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, gripperForce * deltaTime);
                }
            }

            if (m_mapObjectMinDistance.size() > 0)
            {
                AZ_TracePrintf("GripperComponent", "Apply the gripper velocity to the object(s) grasped");
                AZ::Vector3 gripperVelocity;
                Physics::RigidBodyRequestBus::EventResult(gripperVelocity, GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
                for ([[maybe_unused]] auto& [objectEntityId, minDistance] : m_mapObjectMinDistance)
                {
                    Physics::RigidBodyRequestBus::Event(objectEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, gripperVelocity);
                }
            }

            if (m_visualise)
            {
                Visualise();
                m_raycastPoint.clear();
            }
        }
    }
} // namespace ROS2