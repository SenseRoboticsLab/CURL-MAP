//
// Created by zkc on 18/08/23.
//
#include "ConformalMappingSPH.h"
#include "CurlMapGeneration.h"
#include "types.h"
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <thread>

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_generation_node");
    ros::NodeHandle nh;
    if (argc < 2) {
        std::cerr << "Please provide a yaml file!" << std::endl;
        return -1;
    }
    // load parameters
    std::shared_ptr<SH_TABLE_CONFIG<BT>> SH_table_config_ptr = std::make_shared<SH_TABLE_CONFIG<BT>>();
    load_SH_table_config<BT>(argv[1], *SH_table_config_ptr);
    curl::getSH_table(*SH_table_config_ptr);
    curl::get_update_idx(*SH_table_config_ptr);
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr = std::make_shared<DIRECT_METHOD_CONFIG>();
    load_direct_method_config(argv[1], *direct_method_config_ptr);
    std::shared_ptr<CONFORMAL_MAPPING_CONFIG> conformal_mapping_config_ptr =
        std::make_shared<CONFORMAL_MAPPING_CONFIG>();
    load_conformal_mapping_config(argv[1], *conformal_mapping_config_ptr);
    std::shared_ptr<MAP_GENERATION_CONFIG> map_generation_config_ptr = std::make_shared<MAP_GENERATION_CONFIG>();
    load_map_generation(argv[1], *map_generation_config_ptr);
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr = std::make_shared<DEBUG_CONFIG>();
    load_debug_config(argv[1], *debug_config_ptr);
    if (argc == 3) {
        debug_config_ptr->seq = argv[2];
    }
    std::cout << "Sequence: " << debug_config_ptr->seq << std::endl;
    std::shared_ptr<AABB_CONFIG> aabb_config_ptr = std::make_shared<AABB_CONFIG>();
    load_aabb_config(argv[1], *aabb_config_ptr);
    // initilize conformal mapping class
    std::shared_ptr<ConformalMappingSPH<BT>> conformal_mapping_ptr = std::make_shared<ConformalMappingSPH<BT>>(
        conformal_mapping_config_ptr, SH_table_config_ptr, direct_method_config_ptr, aabb_config_ptr, debug_config_ptr);
    // initilize map generation class
    CurlMapGeneration<BT> curl_map_generation(&nh, map_generation_config_ptr, SH_table_config_ptr,
                                              direct_method_config_ptr, debug_config_ptr, conformal_mapping_ptr);
    std::thread t_map_generation(&CurlMapGeneration<BT>::run, &curl_map_generation);
    std::thread t_open3d_display(&CurlMapGeneration<BT>::open3d_display_thread, &curl_map_generation);
    t_map_generation.join();
    return 0;
}