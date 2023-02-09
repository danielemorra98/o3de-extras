/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <URDFMetadataComponent.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>


namespace ROS2
{
    void URDFMetadataComponent::Activate()
    {
    }

    void URDFMetadataComponent::Deactivate()
    {
    }

    void URDFMetadataComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("URDFMetadata"));
    }

    void URDFMetadataComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<URDFMetadataComponent, AZ::Component>()
                ->Version(0)
                ->Field("HierarchyMap", &URDFMetadataComponent::m_hierarchyMap);
        }
    }

    void URDFMetadataComponent::SetHierarchy(const AZStd::unordered_map<AZ::Name, AZ::EntityId> & hierarchyMap)
    {
        m_hierarchyMap = hierarchyMap;
    }

    AZStd::unordered_map<AZ::Name, AZ::EntityId> URDFMetadataComponent::GetHierarchy()
    {
        return m_hierarchyMap;
    }

} // namespace ROS2
