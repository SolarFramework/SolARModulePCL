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

#include "SolARPCLHelper.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

pcl::PointCloud<PointXYZ>::Ptr SolARPCLHelper::solar2pclPointCloud( const SRef<PointCloud>& inPointCloud )
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr outPointCloudPCL( new pcl::PointCloud<pcl::PointXYZ> );

//    const auto& in_points = inPointCloud->getConstPointCloud();

//    for( const auto& pt : in_points )
//    {
//        outPointCloudPCL->push_back( { pt.x(), pt.y(), pt.z() } );
//    }

//    return outPointCloudPCL;
}

}
}
}
