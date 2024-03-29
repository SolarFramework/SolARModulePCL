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

#include "SolARPCFilterCentroidPCL.h"
#include "SolARPCLHelper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PCL::SolARPCFilterCentroid)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

SolARPCFilterCentroid::SolARPCFilterCentroid():ConfigurableBase(xpcf::toUUID<SolARPCFilterCentroid>())
{
    addInterface<api::pointCloud::IPCFilterCentroid>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloat( "radiusThreshold", m_radiusThreshold );
}

FrameworkReturnCode SolARPCFilterCentroid::filter(const SRef<PointCloud> inPointCloud, const SRef<Point3Df> centroid, SRef<PointCloud>& outPointCloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_points_pcl = SolARPCLHelper::solar2pclPointCloud( inPointCloud );
	outPointCloud = xpcf::utils::make_shared<PointCloud>();

    const pcl::PointXYZ ref_point{ centroid->x(), centroid->y(), centroid->z() };

    for( const auto& pt : *in_points_pcl )
    {
        if( pcl::geometry::distance( pt, ref_point ) <= m_radiusThreshold )
			outPointCloud->addPoint(CloudPoint(pt.x, pt.y, pt.z));
    }

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
