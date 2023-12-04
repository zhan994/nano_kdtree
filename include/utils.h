/**
 * \file utils.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief utils
 * \version 0.1
 * \date 2023-12-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>

double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}