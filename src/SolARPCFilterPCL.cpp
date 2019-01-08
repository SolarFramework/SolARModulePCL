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

#include "SolARPCFilterPCL.h"
#include "SolARPCLHelper.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PCL::PCFilter)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

PCFilter::PCFilter():ConfigurableBase(xpcf::toUUID<PCFilter>())
{
    addInterface<api::pointCloud::IPCFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloat( "leafSize", m_leafSize );
    params->wrapFloat( "depthMax", m_depthMax );
}

FrameworkReturnCode PCFilter::filter(const SRef<PointCloud> inPointCloud, SRef<PointCloud>& outPointCloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr inPointCloudPCL = SolARPCLHelper::solar2pclPointCloud( inPointCloud );

    if (outPointCloud == nullptr)
        outPointCloud = xpcf::utils::make_shared<PointCloud>();

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud( inPointCloudPCL );
    vg.setLeafSize( m_leafSize, m_leafSize, m_leafSize );
    vg.setFilterFieldName( "z" );
    vg.setFilterLimits( 0.f, m_depthMax );
    vg.filter( *inPointCloudPCL );

    *outPointCloud = SolARPCLHelper::pcl2solarPointCloud( inPointCloudPCL );

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
