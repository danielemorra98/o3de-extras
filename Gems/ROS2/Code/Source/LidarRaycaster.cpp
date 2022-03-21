#include "LidarRaycaster.h"
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/smart_ptr/make_shared.h>

using namespace ROS2;

LidarRaycaster::LidarRaycaster()
{
}

// A simplified, non-optimized first version. TODO - generalize results (fields)
void LidarRaycaster::PerformRaycast(const AZ::Vector3 &start, const AZStd::vector<AZ::Vector3> &directions,
                                    float distance, AZStd::vector<AZ::Vector3> &results)
{
    results.clear();
    AzPhysics::SceneQueryRequests requests;
    for (const AZ::Vector3 &direction : directions)
    {   // NOTE - performance-wise, consider reusing requests
        AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
        request->m_start = start;
        request->m_direction = direction;
        request->m_distance = distance;
        request->m_reportMultipleHits = false;
        requests.emplace_back(AZStd::move(request));
    }

    auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
    if (AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
                sceneHandle != AzPhysics::InvalidSceneHandle)
    {
        auto requestResults = sceneInterface->QuerySceneBatch(sceneHandle, requests);
        for (const auto &requestResult : requestResults)
        {   // TODO - check flag for SceneQuery::ResultFlags::Position
            if (requestResult.m_hits.size() > 0)
            {
                results.push_back(requestResult.m_hits[0].m_position);
            }
        }
    }
    else { /* TODO - handle error */ }
}