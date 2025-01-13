//
// Created by zkc on 04/05/23.
//

#include "ConformalMappingSPH.h"
#include "CurlTracking.h"
#include "types.h"
#include <memory>
#include <ros/ros.h>
#include <thread>

int main(int argc, char **argv) {
    if (argc == 1) {
        std::cerr << "Please provide config.yaml file!" << std::endl;
    }
    ros::init(argc, argv, "curl_odometry");
    ros::NodeHandle nh;
    // load parameters
    // initilize the SPH table
    std::shared_ptr<SH_TABLE_CONFIG<BT>> SH_table_config_ptr = std::make_shared<SH_TABLE_CONFIG<BT>>();
    load_SH_table_config<BT>(argv[1], *SH_table_config_ptr);
    curl::getSH_table(*SH_table_config_ptr);
    curl::get_update_idx(*SH_table_config_ptr);
    std::shared_ptr<DIRECT_METHOD_CONFIG> direct_method_config_ptr = std::make_shared<DIRECT_METHOD_CONFIG>();
    load_direct_method_config(argv[1], *direct_method_config_ptr);
    std::shared_ptr<LOCAL_BA_CONFIG> local_BA_config_ptr = std::make_shared<LOCAL_BA_CONFIG>();
    load_local_BA_config(argv[1], *local_BA_config_ptr);
    std::shared_ptr<CONFORMAL_MAPPING_CONFIG> conformal_mapping_config_ptr =
        std::make_shared<CONFORMAL_MAPPING_CONFIG>();
    load_conformal_mapping_config(argv[1], *conformal_mapping_config_ptr);
    std::shared_ptr<DEBUG_CONFIG> debug_config_ptr = std::make_shared<DEBUG_CONFIG>();
    load_debug_config(argv[1], *debug_config_ptr);
    if (argc == 3) {
        debug_config_ptr->seq = argv[2];
    }
    std::cout << "Sequence: " << debug_config_ptr->seq << std::endl;
    std::shared_ptr<AABB_CONFIG> aabb_config_ptr = std::make_shared<AABB_CONFIG>();
    load_aabb_config(argv[1], *aabb_config_ptr);
    std::shared_ptr<CURL_TRACKING_CONFIG> curl_tracking_config_ptr = std::make_shared<CURL_TRACKING_CONFIG>();
    load_curl_tracking(argv[1], *curl_tracking_config_ptr);
    // initlize the conformal map (database)
    std::shared_ptr<ConformalMappingSPH<BT>> conformal_mapping_ptr =
        std::make_shared<ConformalMappingSPH<BT>>(&nh, conformal_mapping_config_ptr, SH_table_config_ptr,
                                                  direct_method_config_ptr, aabb_config_ptr, debug_config_ptr);
    // initialize the map manager
    //    std::shared_ptr<CurlMapUpdate<BT>> curl_map_update_ptr =
    //        std::make_shared<CurlMapUpdate<BT>>(&nh, curl_tracking_config_ptr, debug_config_ptr,
    //        conformal_mapping_ptr);
    // initlize the tracking
    std::shared_ptr<CurlTracking<BT>> curl_tracking_ptr = std::make_shared<CurlTracking<BT>>(
        &nh, curl_tracking_config_ptr, aabb_config_ptr, SH_table_config_ptr, debug_config_ptr, conformal_mapping_ptr);
    //    initlize the local bundle adjustment
    //    std::shared_ptr<CurlLocalBA<BT>> curl_local_ba_ptr =
    //        std::make_shared<CurlLocalBA<BT>>(&nh, local_BA_config_ptr, direct_method_config_ptr, SH_table_config_ptr,
    //                                          conformal_mapping_ptr, debug_config_ptr);
    // initlize the multi-thread
    //    std::thread t_tf_publish(&TfPublisher::run, tf_publisher_ptr);
    std::thread t_tracking(&CurlTracking<BT>::run, curl_tracking_ptr);
    std::thread t_tracking_add_landmark(&CurlTracking<BT>::add_landmark_thread, curl_tracking_ptr);
    std::thread t_tracking_update_sph_coeff(&CurlTracking<BT>::update_sph_coeff_thread, curl_tracking_ptr);
    //    std::thread t_local_ba(&CurlLocalBA<BT>::run, curl_local_ba_ptr);

    ros::spin();
    //    t_tf_publish.join();
    t_tracking.join();
    //    t_tracking_add_landmarl.join();
    //    t_tracking_update_sph_coeff.join();
    //    t_local_ba.join();
    return 0;
}