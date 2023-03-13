#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace ROS2
{
    //! Component responsible for controlling a Gripper
    class GripperComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        enum class Gripper
        {
            Vacuum
        };

        AZ_COMPONENT(GripperComponent, "{b07c684e-8680-4e6d-88f4-40376adb0b58}", AZ::Component);

        GripperComponent() = default;
        ~GripperComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void SetGripperState(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        AZStd::vector<AZStd::pair<AZ::EntityId,AZ::Vector3>> PerformRaycast();
        AZStd::vector<AZ::Vector3> GetGripperRayRotations();
        AzPhysics::SceneHandle GetPhysicsSceneFromEntityId(const AZ::EntityId& entityId);
        AZStd::vector<AZ::Vector3> RotationsToDirections(const AZStd::vector<AZ::Vector3>& rotations, const AZ::Transform& rootTransform);
        AZ::Vector3 ComputeForce(const float gripperDistance, const AZ::Vector3 normalizeDirection);
        void Visualise();
        Gripper m_gripperType = Gripper::Vacuum;
        bool m_gripperState{false};
        bool m_visualise{false};
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_ROS2gripperService;
        AZStd::vector<AZ::Vector3> m_raycastPoint;
        AZStd::vector<AZ::Vector3> m_rayRelativeRotations;
        AZStd::unordered_map<AZ::EntityId,float> m_mapObjectMinDistance;
        AzPhysics::SceneHandle m_sceneHandle{ AzPhysics::InvalidSceneHandle };
        float m_range{0.0f};
        float m_maxGripperForce{0.0f};
        float m_resolution{2.0f};
        float m_coneAngle{0.0f};
        unsigned int m_ignoreLayerCollision{10};

        // Used only when visualisation is on - points differ since they are in global transform as opposed to local
        AZStd::vector<AZ::Vector3> m_visualisationPoints;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
    };
} // namespace ROS2
