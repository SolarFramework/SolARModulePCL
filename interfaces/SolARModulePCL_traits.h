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
/**
 * @namespace SolAR::MODULES::PCL
 * @brief <B>Provides a set of function to process point cloud based on the point clouds library: https://pointclouds.org </B>
 * <TT>UUID: bc1a5b44-d022-4234-8f7a-7e2b72763bad</TT>
 *
 */
namespace PCL {
class SolARPointCloudLoader;
class SolARPCFilter;
class SolARPCFilterCentroid;
class SolARICP;
class SolARICPNormals;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::SolARPointCloudLoader,
                             "0415dec1-d5e5-4497-8fd0-a86e21fbc5d5",
                             "SolARPointCloudLoader",
                             "A component based on PCL to load a point cloud from a file")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::SolARPCFilter,
                             "14f2f201-39f3-4871-94a1-6e9dc852d3dd",
                             "SolARPCFilter",
                             "A component based on PCL to filter a point cloud")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::SolARPCFilterCentroid,
                             "c90122f0-b432-4809-bf11-85165298b82d",
                             "SolARPCFilterCentroid",
                             "A component based on PCL to filter a point cloud according to a given centroid")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::SolARICP,
                             "a9b28074-f29b-480a-bbf4-81085a5b64cf",
                             "SolARICP",
                             "A Iterative Closest Point component based on PCL")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PCL::SolARICPNormals,
                             "e3e0e76f-6cf3-4848-b267-633550a52633",
                             "SolARICPNormals",
                             "A Iterative Closest Point component based on PCL which use normals")

#endif // SOLARMODULEPCL_TRAITS_H
