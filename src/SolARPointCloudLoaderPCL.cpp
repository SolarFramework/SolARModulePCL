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

#include "SolARPointCloudLoaderPCL.h"
#include "SolARPCLHelper.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PCL::SolARPointCloudLoader)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

SolARPointCloudLoader::SolARPointCloudLoader():ConfigurableBase(xpcf::toUUID<SolARPointCloudLoader>())
{
	declareInterface<api::input::files::IPointCloudLoader>(this);
	declareProperty("filePath", m_filePath);
	LOG_DEBUG("SolARPointCloudLoader constructor");
}

FrameworkReturnCode SolARPointCloudLoader::load(SRef<PointCloud>& pointCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPCL( new pcl::PointCloud<pcl::PointXYZ> );

    int res = 0;
    if( boost::algorithm::ends_with( m_filePath, ".pcd" ) )
        res = pcl::io::loadPCDFile( m_filePath, *pointCloudPCL );
    else if( boost::algorithm::ends_with( m_filePath, ".ply" ) )
        res = pcl::io::loadPLYFile( m_filePath, *pointCloudPCL );
    else
    {
        LOG_ERROR("file extension not managed for file {} (only .pcd and .ply supported for now)", m_filePath );
        return FrameworkReturnCode::_STOP;
    }

    if( res < 0 )
    {
        LOG_ERROR("cannot parse file {} (return code:{})", m_filePath, res );
        return FrameworkReturnCode::_STOP;
    }

    if( pointCloud == nullptr )
        pointCloud = xpcf::utils::make_shared<PointCloud>();

    *pointCloud = SolARPCLHelper::pcl2solarPointCloud( pointCloudPCL );

    LOG_INFO("successfully loaded pointcloud file {} - {} points",m_filePath,pointCloud->getNbPoints());
    return FrameworkReturnCode::_SUCCESS;
}
}
}
}
