#include <ROS2/Manipulation/MotorConfiguration.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void MotorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MotorConfiguration>()
                ->Version(1)
                ->Field("PID configuration", &MotorConfiguration::m_motorPID)
                ->Field("Motor entity", &MotorConfiguration::m_motorEntityId);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<MotorConfiguration>("PID configuration", "Configures a PID controller")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorConfiguration::m_motorPID,
                        "PID parameters",
                        "PID parameters")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorConfiguration::m_motorEntityId,
                        "Motor Entity",
                        "The entity which has an actuated joint attached as component");
            }
        }
    }
} // namespace ROS2