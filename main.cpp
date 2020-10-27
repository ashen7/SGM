/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/27/2020 04:27:50 PM
 *       Revision:  none
 *       Compiler:  g++
 *
 *         Author:  yipeng
 *   Organization:  
 *
 * =====================================================================================
 */

#include <cstdio>
#include <cstdint>
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "semi_global_matching.h"

DEFINE_string(left_image,    "data/cone/img0.png", "left image path");
DEFINE_string(right_image,   "data/cone/img1.png", "right image path");
DEFINE_int32(min_disp,       0,                    "min disparity");
DEFINE_int32(max_disp,       64,                   "min disparity");

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "log/");
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_colorlogtostderr = true;

    // 读取左图右图
    std::string left_path = FLAGS_left_image;
    std::string right_path = FLAGS_right_image;

    cv::Mat left_image = cv::imread(left_path, cv::IMREAD_COLOR);
    cv::Mat left_gray_image = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
    cv::Mat right_gray_image = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

    if (left_gray_image.data == nullptr || right_gray_image.data == nullptr) {
        LOG(ERROR) << "读取图片失败！";
        return -1;
    }
    if (left_gray_image.rows != right_gray_image.rows 
            || left_gray_image.cols != right_gray_image.cols) {
        LOG(ERROR) << "左右图片尺寸不一致！";
        return -1;
    }

    const std::int32_t height = static_cast<std::int32_t>(left_gray_image.rows);
    const std::int32_t width = static_cast<std::int32_t>(left_gray_image.cols);
    const std::int32_t image_size = height * width;

    // 左右影像的灰度数据
    auto left_image_data = std::shared_ptr<std::uint8_t>(new std::uint8_t[image_size], 
                                                         [](std::uint8_t* data) { delete []data; });
    auto right_image_data = std::shared_ptr<std::uint8_t>(new std::uint8_t[image_size], 
                                                          [](std::uint8_t* data) { delete []data; });
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            left_image_data.get()[i * width + j] = left_gray_image.at<std::uint8_t>(i, j);
            right_image_data.get()[i * width + j] = right_gray_image.at<std::uint8_t>(i, j);
        }
    }

    // SGM匹配参数设计
    SemiGlobalMatching::SGMOption sgm_option;
    // 聚合路径数
    sgm_option.num_paths = 8;
    // 候选视差范围
    sgm_option.min_disparity = FLAGS_min_disp;
    sgm_option.max_disparity = FLAGS_max_disp;
    // census窗口类型
    sgm_option.census_size = SemiGlobalMatching::Census5x5;
    // 一致性检查
    sgm_option.is_check_lr = true;
    sgm_option.lr_check_thresh = 1.0f;
    // 唯一性约束
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ratio = 0.99;
    // 剔除小连通区
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_aera = 50;
    // 惩罚项P1、P2
    sgm_option.p1 = 10;
    sgm_option.p2_init = 150;
    // 视差图填充
    // 视差图填充的结果并不可靠，若工程，不建议填充，若科研，则可填充
    sgm_option.is_fill_holes = false;

    LOG(INFO) << "h = " << height << ", w = " << width << ", " << "d = [" 
              << sgm_option.min_disparity << ", " << sgm_option.max_disparity << "]\n";

    // 定义SGM匹配类实例
    SemiGlobalMatching sgm;
    // 初始化
	LOG(INFO) << "SGM Initializing...";
    auto start = std::chrono::steady_clock::now();
    if (!sgm.Initialize(height, width, sgm_option)) {
        LOG(ERROR) << "SGM初始化失败！" << std::endl;
        return -1;
    }
    auto end = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG(INFO) << "SGM Initializing Done! Timing : " << cost_time.count() / 1000.0 << "s";

    // 匹配
	LOG(INFO) << "SGM Matching...";
    start = std::chrono::steady_clock::now();
    // disparity数组保存子像素的视差结果
    auto disparity = std::shared_ptr<float>(new std::float[image_size], 
                                            [](std::float* data) { delete []data; });
    if (!sgm.Match(left_image_data, right_image_data, disparity)) {
        LOG(ERROR) << "SGM匹配失败！";
        return -1;
    }
    end = std::chrono::steady_clock::now();
    cost_time = duration_cast<std::chrono::milliseconds>(end - start);
    LOG(INFO) << "SGM Matching...Done! Timing : " << cost_time.count() / 1000.0 << "s";

	//// 显示视差图
    //// 注意，计算点云不能用disp_mat的数据，它是用来显示和保存结果用的。计算点云要用上面的disparity数组里的数据，是子像素浮点数
    //cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    //float min_disp = width, max_disp = -width;
    //for (std::int32_t i = 0; i < height; i++) {
    //    for (std::int32_t j = 0; j < width; j++) {
    //        const float disp = disparity[i * width + j];
    //        if (disp != Invalid_Float) {
    //            min_disp = std::min(min_disp, disp);
    //            max_disp = std::max(max_disp, disp);
    //        }
    //    }
    //}
    //for (std::int32_t i = 0; i < height; i++) {
    //    for (std::int32_t j = 0; j < width; j++) {
    //        const float disp = disparity[i * width + j];
    //        if (disp == Invalid_Float) {
    //            disp_mat.data[i * width + j] = 0;
    //        }
    //        else {
    //            disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
    //        }
    //    }
    //}

    //cv::imshow("视差图", disp_mat);
    //cv::Mat disp_color;
    //applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    //cv::imshow("视差图-伪彩", disp_color);

    //// 保存结果
    //std::string disp_map_path = argc[1]; disp_map_path += ".d.png";
    //std::string disp_color_map_path = argc[1]; disp_color_map_path += ".c.png";
    //cv::imwrite(disp_map_path, disp_mat);
    //cv::imwrite(disp_color_map_path, disp_color);

    //cv::waitKey(0);

    google::ShutDownCommandLineFlags();
    google::ShutdownGoogleLogging();

    return 0;
}

