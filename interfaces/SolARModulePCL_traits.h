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

#ifndef SOLARMODULEPCL_TRAITS_H
#define SOLARMODULEPCL_TRAITS_H

#include "xpcf/api/IComponentManager.h"

namespace SolAR {
namespace MODULES {
namespace PCL {
class PointCloudLoader;
class PCFilter;
class PCFilterCentroid;
class ICP;
class ICPNormals;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::PointCloudLoader,
                             "0415dec1-d5e5-4497-8fd0-a86e21fbc5d5",
                             "PointCloudLoader",
                             "A component based on PCL to load a point cloud from a file")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::PCFilter,
                             "14f2f201-39f3-4871-94a1-6e9dc852d3dd",
                             "PCFilter",
                             "A component based on PCL to filter a point cloud")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::PCFilterCentroid,
                             "c90122f0-b432-4809-bf11-85165298b82d",
                             "PCFilterCentroid",
                             "A component based on PCL to filter a point cloud according to a given centroid")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::ICP,
                             "a9b28074-f29b-480a-bbf4-81085a5b64cf",
                             "ICP",
                             "A Iterative Closest Point component based on PCL")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::ICPNormals,
                             "e3e0e76f-6cf3-4848-b267-633550a52633",
                             "ICPNormals",
                             "A Iterative Closest Point component based on PCL which use normals")

#endif // SOLARMODULEPCL_TRAITS_H
