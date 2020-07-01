/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <iostream>

#include "xpcf/module/ModuleFactory.h"
#include "SolARModulePCL_traits.h"

#include "SolARPointCloudLoaderPCL.h"
#include "SolARICPNormalsPCL.h"
#include "SolARICPPCL.h"
#include "SolARPCFilterCentroidPCL.h"
#include "SolARPCFilterPCL.h"


namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("bc1a5b44-d022-4234-8f7a-7e2b72763bad", "SolARModulePCL", "A module to handle Point Cloud Library features")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::MODULES::PCL::SolARPointCloudLoader>(componentUUID,interfaceRef);
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::PCL::SolARICPNormals>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::PCL::SolARICP>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::PCL::SolARPCFilter>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::PCL::SolARPCFilterCentroid>(componentUUID,interfaceRef);
    }
    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::PCL::SolARPointCloudLoader)
XPCF_ADD_COMPONENT(SolAR::MODULES::PCL::SolARICPNormals)
XPCF_ADD_COMPONENT(SolAR::MODULES::PCL::SolARICP)
XPCF_ADD_COMPONENT(SolAR::MODULES::PCL::SolARPCFilter)
XPCF_ADD_COMPONENT(SolAR::MODULES::PCL::SolARPCFilterCentroid)
XPCF_END_COMPONENTS_DECLARATION
