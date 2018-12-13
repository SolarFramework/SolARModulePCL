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
}

FrameworkReturnCode ICP::estimate(const SRef<PointCloud> sensorPointCloud,
                                  const SRef<PointCloud> worldPointCloud,
                                  Transform3Df& pose,
                                  const Transform3Df initialPose) {
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
