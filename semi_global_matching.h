#pragma once
#include <vector>

class SemiGlobalMatching {
public:
	SemiGlobalMatching();
	~SemiGlobalMatching();

	/** \brief Census���ڳߴ����� */
	enum CensusSize {
		Census5x5 = 0,
		Census9x7
	};

	/** \brief SGM�����ṹ�� */
	struct SGMOption {
		std::uint8_t	num_paths;			// �ۺ�·���� 4 and 8
		std::int32_t    min_disparity;		// ��С�Ӳ�
		std::int32_t	max_disparity;		// ����Ӳ�

		CensusSize census_size;		// census���ڳߴ�

		bool	is_check_unique;	// �Ƿ���Ψһ��
		float	uniqueness_ratio;	// Ψһ��Լ����ֵ ����С����-����С����)/��С���� > ��ֵ Ϊ��Ч����

		bool	is_check_lr;		// �Ƿ�������һ����
		float	lr_check_thresh;	// ����һ����Լ����ֵ

		bool	is_remove_speckles;	// �Ƿ��Ƴ�С����ͨ��
		int		min_speckle_aera;	// ��С����ͨ���������������

		bool	is_fill_holes;		// �Ƿ�����Ӳ�ն�

		// P1,P2 
		// P2 = P2_init / (Ip-Iq)
		std::int32_t  p1;			// �ͷ������P1
		std::int32_t  p2_init;		// �ͷ������P2

		SGMOption(): num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
		             is_check_unique(true), uniqueness_ratio(0.95f),
		             is_check_lr(true), lr_check_thresh(1.0f),
		             is_remove_speckles(true), min_speckle_aera(20),
		             is_fill_holes(true),
		             p1(10), p2_init(150) { }
	};

	/**
	 * \brief ��ĳ�ʼ�������һЩ�ڴ��Ԥ���䡢������Ԥ���õ�
	 * \param height	���룬�������Ӱ���
	 * \param width		���룬�������Ӱ���
	 * \param option	���룬SemiGlobalMatching����
	 */
	bool Initialize(const std::int32_t& height, const std::int32_t& width, const SGMOption& option);

	/**
	 * \brief ִ��ƥ��
	 * \param img_left	���룬��Ӱ������ָ�� 
	 * \param img_right	���룬��Ӱ������ָ��
	 * \param disp_left	�������Ӱ���Ӳ�ͼָ�룬Ԥ�ȷ����Ӱ��ȳߴ���ڴ�ռ�
	 */
	bool Match(const std::uint8_t* img_left, const std::uint8_t* img_right, float* disp_left);

	/**
	 * \brief ����
	 * \param height	���룬�������Ӱ���
	 * \param width		���룬�������Ӱ���
	 * \param option	���룬SemiGlobalMatching����
	 */
	bool Reset(const std::uint32_t& height, const std::uint32_t& width, const SGMOption& option);

private:
	/** \brief Census�任 */
	void CensusTransform() const;

	/** \brief ���ۼ���	 */
	void ComputeCost() const;

	/** \brief ���۾ۺ�	 */
	void CostAggregation() const;

	/** \brief �Ӳ����	 */
	void ComputeDisparity() const;

	/** \brief �Ӳ����	 */
	void ComputeDisparityRight() const;

	/** \brief һ���Լ��	 */
	void LRCheck();

	/** \brief �Ӳ�ͼ��� */
	void FillHolesInDispMap();

	/** \brief �ڴ��ͷ�	 */
	void Release();

	/** \brief SGM����	 */
	SGMOption option_;

	/** \brief Ӱ���	 */
	std::int32_t height_;

	/** \brief Ӱ���	 */
	std::int32_t width_;

	/** \brief ��Ӱ������	 */
	const std::uint8_t* left_image_;

	/** \brief ��Ӱ������	 */
	const std::uint8_t* right_image_;
	
	/** \brief ��Ӱ��censusֵ	*/
	void* left_census_;
	
	/** \brief ��Ӱ��censusֵ	*/
	void* right_census_;
	
	/** \brief ��ʼƥ�����	*/
	std::uint8_t* cost_init_;
	
	/** \brief �ۺ�ƥ�����	*/
	uint16* cost_aggr_;

	// �K �� �L   5  3  7
	// ��    ��	 1    2
	// �J �� �I   8  4  6
	/** \brief �ۺ�ƥ�����-����1	*/
	std::uint8_t* cost_aggr_1_;
	/** \brief �ۺ�ƥ�����-����2	*/
	std::uint8_t* cost_aggr_2_;
	/** \brief �ۺ�ƥ�����-����3	*/
	std::uint8_t* cost_aggr_3_;
	/** \brief �ۺ�ƥ�����-����4	*/
	std::uint8_t* cost_aggr_4_;
	/** \brief �ۺ�ƥ�����-����5	*/
	std::uint8_t* cost_aggr_5_;
	/** \brief �ۺ�ƥ�����-����6	*/
	std::uint8_t* cost_aggr_6_;
	/** \brief �ۺ�ƥ�����-����7	*/
	std::uint8_t* cost_aggr_7_;
	/** \brief �ۺ�ƥ�����-����8	*/
	std::uint8_t* cost_aggr_8_;

	/** \brief ��Ӱ���Ӳ�ͼ	*/
	float* left_disp_;
	/** \brief ��Ӱ���Ӳ�ͼ	*/
	float* right_disp_;

	/** \brief �Ƿ��ʼ����־	*/
	bool is_initialized_;

	/** \brief �ڵ������ؼ�	*/
	std::vector<std::pair<int, int>> occlusions_;
	/** \brief ��ƥ�������ؼ�	*/
	std::vector<std::pair<int, int>> mismatches_;
};

