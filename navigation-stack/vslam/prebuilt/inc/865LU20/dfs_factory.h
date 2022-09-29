/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RV_DFS_FACTORY_H
#define RV_DFS_FACTORY_H

#include <memory>
#include <rv.h>
#include <rvDFS.h>

#include "rv_dfs_base.h"

namespace rv_dfs
{

	RV_API std::shared_ptr<DFSBase> CreateDFSbase(const rvDFSMode& dfs_mode);

}

#endif
