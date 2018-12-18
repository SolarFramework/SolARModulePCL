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

#ifndef SOLAR_ICPNORMALS_PCL_H
#define SOLAR_ICPNORMALS_PCL_H

#include <vector>
#include "api/solver/pose/I3DTransformFinderFrom3D3D.h"
#include "SolARPCLAPI.h"
#include "xpcf/component/ConfigurableBase.h"


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

/**
 * @class ICP
 * @brief This component filters a point cloud according to a given centroid based on Point Cloud Library (PCL).
 */
class SOLARPCL_EXPORT_API ICPNormals : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformFinderFrom3D3D
{
public:
    ICPNormals();
    ~ICPNormals()= default;

    /// @brief Estimates depth sensor pose from a set of 3D points captured by the depth sensor and defined in the depth sensor coordinate and a point cloud representing the real world geometry.
    /// @param[in] sourcePointCloud, a point cloud captured by the depth sensor defined in the depth sensor coordinate system.
    /// @param[in] targetPointCloud, a point cloud representing the geometry of the real world.
    /// @param[out] pose, depth camera pose (pose of the depth camera defined in world coordinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional), a transform3D to initialize the pose (reducing the convergence time and improving its success).
    FrameworkReturnCode estimate(const SRef<PointCloud> sourcePointCloud,
                                 const SRef<PointCloud> targetPointCloud,
                                 Transform3Df& pose,
                                 const Transform3Df& initialPose = Transform3Df::Identity()) override final;

    void unloadComponent () override final;

private:
    double m_icpPlaneTransformationEpsilon = 1e-8;
    double m_normalEstimationNeighboorhood = 0.02;
};

}
}
}

#endif // SOLAR_ICPNORMALS_PCL_H
