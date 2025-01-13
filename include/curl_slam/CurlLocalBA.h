//
// Created by zkc on 06/06/23.
//

#ifndef SRC_CURLLOCALBA_H
#define SRC_CURLLOCALBA_H

#include "ConformalMappingSPH.h"
#include "DirectMethod.h"
#include "FileReader.h"
#include "RosHandler.h"
#include "Timer.h"
#include "lock_manager.h"
#include "optimization_types.h"
#include "types.h"
#include <Eigen/StdVector>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <utility>
#include <vector>

template <typename BasicType> class CurlLocalBA {
  private:
    std::shared_ptr<LOCAL_BA_CONFIG> local_BA_config_ptr;
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr;
    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> SH_table_config_ptr;
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr;

    std::shared_ptr<ConformalMappingSPH<BasicType>> conformal_mapping_ptr;

  public:
    CurlLocalBA() = delete;
    CurlLocalBA(ros::NodeHandle *nh, std::shared_ptr<LOCAL_BA_CONFIG> _local_BA_config_ptr,
                std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr,
                std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr);
    void run();
};

#endif // SRC_CURLLOCALBA_H
