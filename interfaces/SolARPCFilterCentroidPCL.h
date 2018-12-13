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
using namespace datastructure;
namespace MODULES {
namespace PCL {

/**
 * @class PCFilterCentroid
 * @brief This component filters a point cloud according to a given centroid based on Point Cloud Library (PCL).
 */
class SOLARPCL_EXPORT_API PCFilterCentroid : public org::bcom::xpcf::ConfigurableBase,
    public api::pointCloud::IPCFilterCentroid
{
public:
    PCFilterCentroid();
    ~PCFilterCentroid()= default;

    /// @brief Filter a point cloud according to a given centroid
    /// @param[in] inPointCloud The point cloud to filter
    /// @param[in] centroid The 3D point of reference used to filter the point cloud.
    /// @param[out] outPointCloud The resulting point cloud after filtering
    FrameworkReturnCode filter(const SRef<PointCloud> inPointCloud, const SRef<Point3Df> centroid, SRef<PointCloud>& outPointCloud) const override final;

    void unloadComponent () override;

private:


};

}
}
}

#endif // SOLAR_PCFILTERCENTROID_PCL_H
