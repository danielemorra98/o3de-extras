#pragma once

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
        AZStd::vector<AZStd::pair<AZ::EntityId,float>> PerformRaycast();
        AZStd::vector<AZ::Vector3> GetGripperRayRotations();
        AZStd::vector<AZ::Vector3> RotationsToDirections(const AZStd::vector<AZ::Vector3>& rotations, const AZ::Vector3& rootRotation);
        Gripper m_gripperType = Gripper::Vacuum;
        bool m_gripperState{false};
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_ROS2gripperService;
        // AZStd::vector<AZ::Vector3> m_raycastPoint;
        AZStd::vector<AZ::Vector3> m_rayRotations;
        AzPhysics::SceneHandle m_sceneHandle{ AzPhysics::InvalidSceneHandle };
        float m_range{0.0f};
    };
} // namespace ROS2
