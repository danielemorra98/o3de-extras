/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    //! Configuration for handling of joint's actuator
    struct MotorConfiguration
    {
    public:
        AZ_TYPE_INFO(MotorConfiguration, "{b5d5d94b-c232-4b59-adc1-5bda78e9365b}");

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_motorEntityId;
        AZ::Component* m_hingeComponent;
        AZ::Name m_hingeName;
        Controllers::PidConfiguration m_motorPID;
    };
} // namespace ROS2
