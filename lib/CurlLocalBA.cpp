//
// Created by zkc on 06/06/23.
//
#include "CurlLocalBA.h"

template <typename BasicType>
CurlLocalBA<BasicType>::CurlLocalBA(ros::NodeHandle *nh, std::shared_ptr<LOCAL_BA_CONFIG> _local_BA_config_ptr,
                                    std::shared_ptr<DIRECT_METHOD_CONFIG> _direct_method_config_ptr,
                                    std::shared_ptr<SH_TABLE_CONFIG<BasicType>> _SH_table_config_ptr,
                                    std::shared_ptr<ConformalMappingSPH<BasicType>> _conformal_mapping_ptr,
                                    std::shared_ptr<DEBUG_CONFIG> _debug_config_ptr)
    : local_BA_config_ptr(_local_BA_config_ptr), direct_method_config_ptr(_direct_method_config_ptr),
      SH_table_config_ptr(_SH_table_config_ptr), conformal_mapping_ptr(_conformal_mapping_ptr),
      debug_config_ptr(_debug_config_ptr) {}

template <typename BasicType> void CurlLocalBA<BasicType>::run() {
    int a = 0;
    while (true) {
        a = 5;
        std::cout << "CurlLocalBA thread start" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << a << std::endl;
}

template class CurlLocalBA<BT>; // this is very important
