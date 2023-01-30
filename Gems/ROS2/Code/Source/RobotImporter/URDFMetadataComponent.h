/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Name/Name.h>
#include <AzCore/Component/Component.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2
{
    //! A component responsible for storing the jointComponent tree structure
    //! and each of the joint's name as they are described in URDF
    class URDFMetadataComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(URDFMetadataComponent, "{5d8a6d7d-6847-4e00-9305-b8b848d8b282}", AZ::Component);
        URDFMetadataComponent() = default;

        // Component override
        void Activate() override;
        void Deactivate() override;

        void SetHierarchy(const AZStd::unordered_map<AZ::Name, AZ::EntityId> & hierarchyMap);
        AZStd::unordered_map<AZ::Name, AZ::EntityId> GetHierarchy();

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

        AZStd::unordered_map<AZ::Name, AZ::EntityId> m_hierarchyMap;
    private:

    };
} // namespace ROS2
