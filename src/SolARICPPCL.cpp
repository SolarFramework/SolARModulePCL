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

#include "SolARICPPCL.h"

#include <pcl/registration/icp.h>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PCL::ICP)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

ICP::ICP():ConfigurableBase(xpcf::toUUID<PCFilter>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom3D3D>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapDouble( "transformationEpsilon", m_icpPointTransformationEpsilon );
}

FrameworkReturnCode ICP::estimate(const SRef<PointCloud> sourcePointCloud,
                                  const SRef<PointCloud> targetPointCloud,
                                  Transform3Df& pose,
                                  const Transform3Df& initialPose)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_points_pcl( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_points_pcl( new pcl::PointCloud<pcl::PointXYZ> );

    const auto& source_points = sourcePointCloud->getConstPointCloud();
    const auto& target_points = targetPointCloud->getConstPointCloud();

    for( const auto& pt : source_points )
    {
        source_points_pcl->push_back( { pt.x(), pt.y(), pt.z() } );
    }

    for( const auto& pt : target_points )
    {
        target_points_pcl->push_back( { pt.x(), pt.y(), pt.z() } );
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> picp;
    picp.setEuclideanFitnessEpsilon( m_icpPointTransformationEpsilon );
    picp.setInputSource( source_points_pcl );
    picp.setInputTarget( target_points_pcl );
    picp.align( *source_points_pcl, initialPose.matrix() );

    if( !picp.hasConverged() )
    {
        LOG_ERROR( "point to point registration failed to converge" );
        return FrameworkReturnCode::_STOP;
    }
    else
    {
        pose = picp.getFinalTransformation();
        return FrameworkReturnCode::_SUCCESS;
    }
}

}
}
}
