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

#ifndef SOLAR_PCFILTER_PCL_H
#define SOLAR_PCFILTER_PCL_H

#include <vector>
#include "api/pointCloud/IPCFilter.h"
#include "SolARPCLAPI.h"
#include "xpcf/component/ConfigurableBase.h"


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

/**
 * @class PCFilter
 * @brief This component filters a point cloud based on Point Cloud Library (PCL).
 */
class SOLARPCL_EXPORT_API PCFilter : public org::bcom::xpcf::ConfigurableBase,
    public api::pointCloud::IPCFilter
{
public:
    PCFilter();
    ~PCFilter()= default;

    /// @brief Filter a point cloud
    /// @param[in] inPointCloud The point cloud to filter
    /// @param[out] outPointCloud The resulting point cloud after filtering
    FrameworkReturnCode filter(const SRef<PointCloud> inPointCloud, SRef<PointCloud>& outPointCloud) const override final;

    void unloadComponent () override final;

private:
    float m_leafSize = 0.05f; // in meters <-> defaults to 5cm

};

}
}
}

#endif // SOLAR_PCFILTER_PCL_H
