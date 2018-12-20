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

#ifndef SOLAR_POINTCLOUDLOADER_PCL_H
#define SOLAR_POINTCLOUDLOADER_PCL_H

#include <vector>
#include "api/input/files/IPointCloudLoader.h"
#include "SolARPCLAPI.h"
#include "xpcf/component/ConfigurableBase.h"


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace PCL {

/**
 * @class PointCloudLoader
 * @brief This component loads a point cloud from a file based on Point Cloud Library (PCL).
 */
class SOLARPCL_EXPORT_API PointCloudLoader : public org::bcom::xpcf::ConfigurableBase,
    public api::input::files::IPointCloudLoader
{
public:
    PointCloudLoader();
    ~PointCloudLoader()= default;

    FrameworkReturnCode load(const std::string filepath, SRef<PointCloud>& pointCloud) override final;

    void unloadComponent () override final;

private:


};

}
}
}

#endif // SOLAR_POINTCLOUDLOADER_PCL_H
