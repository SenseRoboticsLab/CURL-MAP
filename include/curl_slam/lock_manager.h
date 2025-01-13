/**
 * @file lock_manager.h
 * @author zkc (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#if !defined(LOCK_MANAGER)
#define LOCK_MANAGER
#include <mutex>
#include <thread>

std::mutex m_queue_lock;

#endif // LOCK_MANAGER
