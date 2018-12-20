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

#include "SolARICPNormalsPCL.h"
#include "SolARPCLHelper.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PCL::ICPNormals)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

inline void _add_normals_2_pointcloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud, const pcl::PointCloud<pcl::PointNormal>::Ptr& pointcloud_with_normals, const double normal_estimation_neighboorhood )
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud( pointcloud );

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nemop;
    nemop.setInputCloud( pointcloud );
    nemop.setNumberOfThreads( 4 );
    nemop.setSearchMethod( tree );
    nemop.setRadiusSearch( normal_estimation_neighboorhood );

    pcl::PointCloud<pcl::Normal> normals;
    nemop.compute( normals );

    pcl::concatenateFields( *pointcloud, normals, *pointcloud_with_normals );
}

ICPNormals::ICPNormals():ConfigurableBase(xpcf::toUUID<ICPNormals>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom3D3D>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapDouble( "transformationEpsilon", m_icpPlaneTransformationEpsilon );
}

FrameworkReturnCode ICPNormals::estimate(const SRef<PointCloud> sourcePointCloud,
                                         const SRef<PointCloud> targetPointCloud,
                                         Transform3Df& pose,
                                         const Transform3Df& initialPose)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_points_pcl = SolARPCLHelper::solar2pclPointCloud( sourcePointCloud );
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_points_pcl = SolARPCLHelper::solar2pclPointCloud( targetPointCloud );

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal> picp;

    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals( new pcl::PointCloud<pcl::PointNormal>() );
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals( new pcl::PointCloud<pcl::PointNormal>() );
    _add_normals_2_pointcloud( source_points_pcl, source_with_normals, m_normalEstimationNeighboorhood );
    _add_normals_2_pointcloud( target_points_pcl, target_with_normals, m_normalEstimationNeighboorhood );

    picp.setEuclideanFitnessEpsilon( m_icpPlaneTransformationEpsilon );
    picp.setInputSource( source_with_normals );
    picp.setInputTarget( target_with_normals );
    picp.align( *source_with_normals, initialPose.matrix() );

    if( !picp.hasConverged() )
    {
        LOG_ERROR( "point to plane registration failed to converge" );
        return FrameworkReturnCode::_STOP;
    }
    else
    {
        pose = picp.getFinalTransformation();
        return FrameworkReturnCode::_SUCCESS;
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
