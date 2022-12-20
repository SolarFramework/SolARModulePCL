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

#include <iostream>
#include <boost/log/core.hpp>
#include <string>


// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/files/IPointCloudLoader.h"
#include "api/display/I3DPointsViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

	try {
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load("SolARTest_ModulePCL_PointCloudLoader_conf.xml") != org::bcom::xpcf::_SUCCESS)
		{
            LOG_ERROR("Failed to load the configuration file SolARTest_ModulePCL_PointCloudLoader_conf.xml")
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
        auto pcLoader = xpcfComponentManager->resolve<input::files::IPointCloudLoader>();
		auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		LOG_INFO("Components created");

		if (!pcLoader || !viewer3DPoints)
		{
			LOG_ERROR("One or more component creations have failed");
			return -1;
		}

		//declaration
		SRef<PointCloud> meshPointCloud;

		// load mesh (.pcd or .ply)
		if(argc==2){
			//change the path of the mesh according to the argument given in parameter
			pcLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->setStringValue(argv[1], 0);
			LOG_INFO("Mesh path override from argument");
		}
		

        if (pcLoader->load(meshPointCloud) != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot load mesh with path : {}", pcLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
			return -1;
		}


		while (true) {
			if (viewer3DPoints->display(meshPointCloud, Transform3Df::Identity(), {}, {}) == FrameworkReturnCode::_STOP)
			{
				LOG_INFO("End of SolARPointCloudLoader test");
				break;
			}
		}
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR ("The following exception has been catch : {}", e.what());
		return -1;
	}
    return 0;
}
