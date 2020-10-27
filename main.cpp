#include <cstdio>
#include <cstdint>
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

#include <opencv2/opencv.hpp>

#include "semi_global_matching.h"

/**
 * \param argc argc[1]:��Ӱ��·�� argc[2]: ��Ӱ��·�� argc[3]: ��С�Ӳ�[��ѡ��Ĭ��0] argc[4]: ����Ӳ�[��ѡ��Ĭ��64]
 * \param eg. ..\Data\cone\im2.png ..\Data\cone\im6.png 0 64
 * \param eg. ..\Data\Reindeer\view1.png ..\Data\Reindeer\view5.png 0 128
 */

int main(int argv, char* argc[]) {
    if (argv < 3) {
        std::cout << "�������٣�������ָ������Ӱ��·����" << std::endl;
        return -1;
    }

    // ��ȡ��ͼ��ͼ
    std::string left_path = argc[1];
    std::string right_path = argc[2];

    cv::Mat left_image = cv::imread(left_path, cv::IMREAD_COLOR);
    cv::Mat left_gray_image = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
    cv::Mat right_gray_image = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

    if (left_gray_image.data == nullptr || right_gray_image.data == nullptr) {
        std::cout << "��ȡͼƬʧ�ܣ�" << std::endl;
        return -1;
    }
    if (left_gray_image.rows != right_gray_image.rows 
            || left_gray_image.cols != right_gray_image.cols) {
        std::cout << "����ͼƬ�ߴ粻һ�£�" << std::endl;
        return -1;
    }

    const std::int32_t height = static_cast<std::int32_t>(right_gray_image.rows);
    const std::int32_t width = static_cast<std::int32_t>(left_gray_image.cols);

    // ����Ӱ��ĻҶ�����
    auto left_image_data = std::shared_ptr<std::uint8_t>(new std::uint8_t[width * height], [](std::uint8_t* data) { delete []data; });
    auto right_image_data = std::shared_ptr<std::uint8_t>(new std::uint8_t[width * height], [](std::uint8_t* data) { delete []data; });
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            left_image_data.get()[i * width + j] = left_gray_image.at<std::uint8_t>(i, j);
            right_image_data.get()[i * width + j] = right_gray_image.at<std::uint8_t>(i, j);
        }
    }

    // SGMƥ��������
    SemiGlobalMatching::SGMOption sgm_option;
    // �ۺ�·����
    sgm_option.num_paths = 8;
    // ��ѡ�ӲΧ
    sgm_option.min_disparity = argv < 4 ? 0 : std::atoi(argc[3]);
    sgm_option.max_disparity = argv < 5 ? 64 : std::atoi(argc[4]);
    // census��������
    sgm_option.census_size = SemiGlobalMatching::Census5x5;
    // һ���Լ��
    sgm_option.is_check_lr = true;
    sgm_option.lr_check_thresh = 1.0f;
    // Ψһ��Լ��
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ratio = 0.99;
    // �޳�С��ͨ��
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_aera = 50;
    // �ͷ���P1��P2
    sgm_option.p1 = 10;
    sgm_option.p2_init = 150;
    // �Ӳ�ͼ���
    // �Ӳ�ͼ���Ľ�������ɿ��������̣���������䣬�����У�������
    sgm_option.is_fill_holes = false;

    printf("w = %d, h = %d, d = [%d,%d]\n\n", width, height, sgm_option.min_disparity, sgm_option.max_disparity);

    // ����SGMƥ����ʵ��
    SemiGlobalMatching sgm;
    // ��ʼ��
	printf("SGM Initializing...\n");
    auto start = std::chrono::steady_clock::now();
    if (!sgm.Initialize(width, height, sgm_option)) {
        std::cout << "SGM��ʼ��ʧ�ܣ�" << std::endl;
        return -2;
    }
    auto end = std::chrono::steady_clock::now();
    auto tt = duration_cast<std::chrono::milliseconds>(end - start);
    printf("SGM Initializing Done! Timing : %lf s\n\n", tt.count() / 1000.0);

    // ƥ��
	printf("SGM Matching...\n");
    start = std::chrono::steady_clock::now();
    // disparity���鱣�������ص��Ӳ���
    auto disparity = std::shared_ptr<std::uint32_t>(new std::uint32_t[width * height], [](std::uint32_t* data) { delete []data; });
    if (!sgm.Match(left_image_data, right_image_data, disparity)) {
        std::cout << "SGMƥ��ʧ�ܣ�" << std::endl;
        return -2;
    }
    end = std::chrono::steady_clock::now();
    tt = duration_cast<std::chrono::milliseconds>(end - start);
    printf("\nSGM Matching...Done! Timing :   %lf s\n", tt.count() / 1000.0);

	// ��ʾ�Ӳ�ͼ
    // ע�⣬������Ʋ�����disp_mat�����ݣ�����������ʾ�ͱ������õġ��������Ҫ�������disparity����������ݣ��������ظ�����
    cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    float min_disp = width, max_disp = -width;
    for (std::int32_t i = 0; i < height; i++) {
        for (std::int32_t j = 0; j < width; j++) {
            const float disp = disparity[i * width + j];
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (std::int32_t i = 0; i < height; i++) {
        for (std::int32_t j = 0; j < width; j++) {
            const float disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
            }
        }
    }

    cv::imshow("�Ӳ�ͼ", disp_mat);
    cv::Mat disp_color;
    applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imshow("�Ӳ�ͼ-α��", disp_color);

    // ������
    std::string disp_map_path = argc[1]; disp_map_path += ".d.png";
    std::string disp_color_map_path = argc[1]; disp_color_map_path += ".c.png";
    cv::imwrite(disp_map_path, disp_mat);
    cv::imwrite(disp_color_map_path, disp_color);

    cv::waitKey(0);

    return 0;
}

