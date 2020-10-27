/*
 * =====================================================================================
 *
 *       Filename:  semi_global_matching.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/27/2020 04:30:07 PM
 *       Revision:  none
 *       Compiler:  g++
 *
 *         Author:  yipeng 
 *   Organization:  
 *
 * =====================================================================================
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

class SemiGlobalMatching {
public:
	SemiGlobalMatching();
	~SemiGlobalMatching();

	/** \brief Census窗口尺寸类型 */
	enum CensusSize {
		Census5x5 = 0,
		Census9x7
	};

	/** \brief SGM参数结构体 */
	struct SGMOption {
		std::uint8_t	num_paths;			// 聚合路径数 4 and 8
		std::int32_t    min_disparity;		// 最小视差
		std::int32_t	max_disparity;		// 最大视差

		CensusSize census_size;		// census窗口尺寸

		bool	is_check_unique;	// 是否检查唯一性
		float	uniqueness_ratio;	// 唯一性约束阈值 （最小代价-次最小代价)/最小代价 > 阈值 为有效像素

		bool	is_check_lr;		// 是否检查左右一致性
		float	lr_check_thresh;	// 左右一致性约束阈值

		bool	is_remove_speckles;	// 是否移除小的连通区
		int		min_speckle_aera;	// 最小的连通区面积（像素数）

		bool	is_fill_holes;		// 是否填充视差空洞

		// P1,P2 
		// P2 = P2_init / (Ip-Iq)
		std::int32_t  p1;			// 惩罚项参数P1
		std::int32_t  p2_init;		// 惩罚项参数P2

		SGMOption(): num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
		             is_check_unique(true), uniqueness_ratio(0.95f),
		             is_check_lr(true), lr_check_thresh(1.0f),
		             is_remove_speckles(true), min_speckle_aera(20),
		             is_fill_holes(true),
		             p1(10), p2_init(150) { }
	};

public:
	/**
	 * \brief 类的初始化，完成一些内存的预分配、参数的预设置等
	 * \param height	输入，核线像对影像高
	 * \param width		输入，核线像对影像宽
	 * \param option	输入，SemiGlobalMatching参数
	 */
	bool Initialize(const std::int32_t& height, const std::int32_t& width, const SGMOption& option);

	/**
	 * \brief 执行匹配
	 * \param left_image	输入，左影像数据指针 
	 * \param right_image	输入，右影像数据指针
	 * \param left_disp	输出，左影像视差图指针，预先分配和影像等尺寸的内存空间
	 */
	bool Match(const std::uint8_t* left_image, const std::uint8_t* right_image, float* left_disp);

	/**
	 * \brief 重设
	 * \param height	输入，核线像对影像高
	 * \param width		输入，核线像对影像宽
	 * \param option	输入，SemiGlobalMatching参数
	 */
	bool Reset(const std::uint32_t& height, const std::uint32_t& width, const SGMOption& option);

private:
	/** \brief Census变换 */
	void CensusTransform() const;

	/** \brief 代价计算	 */
	void ComputeCost() const;

	/** \brief 代价聚合	 */
	void CostAggregation() const;

	/** \brief 视差计算	 */
	void ComputeDisparity() const;

	/** \brief 视差计算	 */
	void ComputeDisparityRight() const;

	/** \brief 一致性检查	 */
	void LRCheck();

	/** \brief 视差图填充 */
	void FillHolesInDispMap();

	/** \brief 内存释放	 */
	void Release();

private:
	/** \brief SGM参数	 */
	SGMOption option_;

	/** \brief 影像高	 */
	std::int32_t height_;

	/** \brief 影像宽	 */
	std::int32_t width_;

	/** \brief 左影像数据	 */
	const std::uint8_t* left_image_;

	/** \brief 右影像数据	 */
	const std::uint8_t* right_image_;
	
	/** \brief 左影像census值	*/
	void* left_census_;
	
	/** \brief 右影像census值	*/
	void* right_census_;
	
	/** \brief 初始匹配代价	*/
	std::uint8_t* cost_init_;
	
	/** \brief 聚合匹配代价	*/
    std::uint16_t* cost_aggr_;

	// ↘ ↓ ↙   5  3  7
	// →    ←	 1    2
	// ↗ ↑ ↖   8  4  6
	/** \brief 聚合匹配代价-方向1	*/
    std::uint8_t* cost_aggr_1_;
	/** \brief 聚合匹配代价-方向2	*/
	std::uint8_t* cost_aggr_2_;
	/** \brief 聚合匹配代价-方向3	*/
	std::uint8_t* cost_aggr_3_;
	/** \brief 聚合匹配代价-方向4	*/
	std::uint8_t* cost_aggr_4_;
	/** \brief 聚合匹配代价-方向5	*/
	std::uint8_t* cost_aggr_5_;
	/** \brief 聚合匹配代价-方向6	*/
	std::uint8_t* cost_aggr_6_;
	/** \brief 聚合匹配代价-方向7	*/
	std::uint8_t* cost_aggr_7_;
	/** \brief 聚合匹配代价-方向8	*/
	std::uint8_t* cost_aggr_8_;

	/** \brief 左影像视差图	*/
	float* left_disp_;
	/** \brief 右影像视差图	*/
	float* right_disp_;

	/** \brief 是否初始化标志	*/
	bool is_initialized_;

	/** \brief 遮挡区像素集	*/
	std::vector<std::pair<int, int>> occlusions_;
	/** \brief 误匹配区像素集	*/
	std::vector<std::pair<int, int>> mismatches_;
};


