/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLAR_PCFILTERCENTROID_PCL_H
#define SOLAR_PCFILTERCENTROID_PCL_H

#include <vector>
#include "api/pointCloud/IPCFilterCentroid.h"
#include "SolARPCLAPI.h"
#include "xpcf/component/ConfigurableBase.h"


namespace SolAR {
namespace MODULES {
namespace PCL {

/**
 * @class SolARPCFilterCentroid
 * @brief This component filters a point cloud according to a given centroid based on Point Cloud Library (PCL).
 */
class SOLARPCL_EXPORT_API SolARPCFilterCentroid : public org::bcom::xpcf::ConfigurableBase,
    public api::pointCloud::IPCFilterCentroid
{
public:
	SolARPCFilterCentroid();
    ~SolARPCFilterCentroid()= default;

    /// @brief Filter a point cloud according to a given centroid
    /// @param[in] inPointCloud The point cloud to filter
    /// @param[in] centroid The 3D point of reference used to filter the point cloud.
    /// @param[out] outPointCloud The resulting point cloud after filtering
    FrameworkReturnCode filter(const SRef<datastructure::PointCloud> inPointCloud, const SRef<datastructure::Point3Df> centroid, SRef<datastructure::PointCloud>& outPointCloud) const override final;

    void unloadComponent () override;

private:
    float m_radiusThreshold = 0.1f; // in meters <-> defaults to 10cm

};

}
}
}

#endif // SOLAR_PCFILTERCENTROID_PCL_H
