#include "sgm_util.h"

#include <algorithm>
#include <cassert>
#include <vector>
#include <queue>

namespace sgm_util {

void census_transform_5x5(const std::uint8_t* source, std::uint32_t* census, 
                          const std::int32_t& height, const std::int32_t& width) {
	if (source == nullptr || census == nullptr || width <= 5 || height <= 5) {
		return;
	}

	// �����ؼ���censusֵ
	for (std::int32_t i = 2; i < height - 2; i++) {
		for (std::int32_t j = 2; j < width - 2; j++) {
			
			// ��������ֵ
			const std::uint8_t gray_center = source[i * width + j];
			
			// ������СΪ5x5�Ĵ������������أ���һ�Ƚ�����ֵ����������ֵ�ĵĴ�С������censusֵ
			std::uint32_t census_val = 0u;
			for (std::int32_t r = -2; r <= 2; r++) {
				for (std::int32_t c = -2; c <= 2; c++) {
					census_val <<= 1;
					const std::uint8_t gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}

			// �������ص�censusֵ
			census[i * width + j] = census_val;		
		}
	}
}

void census_transform_9x7(const std::uint8_t* source, uint64* census, 
                          const std::int32_t& height, const std::int32_t& width) {
	if (source == nullptr || census == nullptr || width <= 9 || height <= 7) {
		return;
	}

	// �����ؼ���censusֵ
	for (std::int32_t i = 4; i < height - 4; i++) {
		for (std::int32_t j = 3; j < width - 3; j++) {

			// ��������ֵ
			const std::uint8_t gray_center = source[i * width + j];

			// ������СΪ5x5�Ĵ������������أ���һ�Ƚ�����ֵ����������ֵ�ĵĴ�С������censusֵ
			uint64 census_val = 0u;
			for (std::int32_t r = -4; r <= 4; r++) {
				for (std::int32_t c = -3; c <= 3; c++) {
					census_val <<= 1;
					const std::uint8_t gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}

			// �������ص�censusֵ
			census[i * width + j] = census_val;
		}
	}
}

std::uint8_t Hamming32(const std::uint32_t& x, const std::uint32_t& y) {
	std::uint32_t dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<std::uint8_t>(dist);
}

std::uint8_t Hamming64(const uint64& x, const uint64& y) {
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<std::uint8_t>(dist);
}


void CostAggregateLeftRight(const std::uint8_t* img_data, const std::int32_t& height, const std::int32_t& width, 
                            const std::int32_t& min_disparity, const std::int32_t& max_disparity,
	                        const std::int32_t& p1, const std::int32_t& p2_init, 
                            const std::uint8_t* cost_init, std::uint8_t* cost_aggr, bool is_forward) {
	assert(width > 0 && height > 0 && max_disparity > min_disparity);

	// �ӲΧ
	const std::int32_t disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// ����(��->��) ��is_forward = true ; direction = 1
	// ����(��->��) ��is_forward = false; direction = -1;
	const std::int32_t direction = is_forward ? 1 : -1;

	// �ۺ�
	for (std::int32_t i = 0u; i < height; i++) {
		// ·��ͷΪÿһ�е���(β,dir=-1)������
		auto cost_init_row = (is_forward) ? (cost_init + i * width * disp_range) : (cost_init + i * width * disp_range + (width - 1) * disp_range);
		auto cost_aggr_row = (is_forward) ? (cost_aggr + i * width * disp_range) : (cost_aggr + i * width * disp_range + (width - 1) * disp_range);
		auto img_row = (is_forward) ? (img_data + i * width) : (img_data + i * width + width - 1);

		// ·���ϵ�ǰ�Ҷ�ֵ����һ���Ҷ�ֵ
		std::uint8_t gray = *img_row;
		std::uint8_t gray_last = *img_row;

		// ·�����ϸ����صĴ������飬������Ԫ����Ϊ�˱���߽��������β����һ����
		std::vector<std::uint8_t> cost_last_path(disp_range + 2, UINT8_MAX);

		// ��ʼ������һ�����صľۺϴ���ֵ���ڳ�ʼ����ֵ
		memcpy(cost_aggr_row, cost_init_row, disp_range * sizeof(std::uint8_t));
		memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(std::uint8_t));
		cost_init_row += direction * disp_range;
		cost_aggr_row += direction * disp_range;
		img_row += direction;

		// ·�����ϸ����ص���С����ֵ
		std::uint8_t mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// �Է����ϵ�2�����ؿ�ʼ��˳��ۺ�
		for (std::int32_t j = 0; j < width - 1; j++) {
			gray = *img_row;
			std::uint8_t min_cost = UINT8_MAX;
			for (std::int32_t d = 0; d < disp_range; d++){
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const std::uint8_t  cost = cost_init_row[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));
				
				const std::uint8_t cost_s = cost + static_cast<std::uint8_t>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);
				
				cost_aggr_row[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// �����ϸ����ص���С����ֵ�ʹ�������
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(std::uint8_t));

			// ��һ������
			cost_init_row += direction * disp_range;
			cost_aggr_row += direction * disp_range;
			img_row += direction;
			
			// ����ֵ���¸�ֵ
			gray_last = gray;
		}
	}
}

void CostAggregateUpDown(const std::uint8_t* img_data, const std::int32_t& width, const std::int32_t& height,
	                     const std::int32_t& min_disparity, const std::int32_t& max_disparity, 
                         const std::int32_t& p1, const std::int32_t& p2_init,
	                     const std::uint8_t* cost_init, std::uint8_t* cost_aggr, bool is_forward) {
	assert(width > 0 && height > 0 && max_disparity > min_disparity);

	// �ӲΧ
	const std::int32_t disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// ����(��->��) ��is_forward = true ; direction = 1
	// ����(��->��) ��is_forward = false; direction = -1;
	const std::int32_t direction = is_forward ? 1 : -1;

	// �ۺ�
	for (std::int32_t j = 0; j < width; j++) {
		// ·��ͷΪÿһ�е���(β,dir=-1)������
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// ·���ϵ�ǰ�Ҷ�ֵ����һ���Ҷ�ֵ
		std::uint8_t gray = *img_col;
		std::uint8_t gray_last = *img_col;

		// ·�����ϸ����صĴ������飬������Ԫ����Ϊ�˱���߽��������β����һ����
		std::vector<std::uint8_t> cost_last_path(disp_range + 2, UINT8_MAX);

		// ��ʼ������һ�����صľۺϴ���ֵ���ڳ�ʼ����ֵ
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(std::uint8_t));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));
		cost_init_col += direction * width * disp_range;
		cost_aggr_col += direction * width * disp_range;
		img_col += direction * width;

		// ·�����ϸ����ص���С����ֵ
		std::uint8_t mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// �Է����ϵ�2�����ؿ�ʼ��˳��ۺ�
		for (std::int32_t i = 0; i < height - 1; i ++) {
			gray = *img_col;
			std::uint8_t min_cost = UINT8_MAX;
			for (std::int32_t d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const std::uint8_t  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const std::uint8_t cost_s = cost + static_cast<std::uint8_t>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// �����ϸ����ص���С����ֵ�ʹ�������
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));

			// ��һ������
			cost_init_col += direction * width * disp_range;
			cost_aggr_col += direction * width * disp_range;
			img_col += direction * width;

			// ����ֵ���¸�ֵ
			gray_last = gray;
		}
	}
}

void CostAggregateDagonal_1(const std::uint8_t* img_data, const std::int32_t& width, const std::int32_t& height,
	                        const std::int32_t& min_disparity, const std::int32_t& max_disparity, 
                            const std::int32_t& p1, const std::int32_t& p2_init,
	                        const std::uint8_t* cost_init, std::uint8_t* cost_aggr, bool is_forward) {
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// �ӲΧ
	const std::int32_t disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// ����(����->����) ��is_forward = true ; direction = 1
	// ����(����->����) ��is_forward = false; direction = -1;
	const std::int32_t direction = is_forward ? 1 : -1;

	// �ۺ�

	// �洢��ǰ�����кţ��ж��Ƿ񵽴�Ӱ��߽�
	std::int32_t current_row = 0;
	std::int32_t current_col = 0;

	for (std::int32_t j = 0; j < width; j++) {
		// ·��ͷΪÿһ�е���(β,dir=-1)������
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// ·�����ϸ����صĴ������飬������Ԫ����Ϊ�˱���߽��������β����һ����
		std::vector<std::uint8_t> cost_last_path(disp_range + 2, UINT8_MAX);

		// ��ʼ������һ�����صľۺϴ���ֵ���ڳ�ʼ����ֵ
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(std::uint8_t));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));

		// ·���ϵ�ǰ�Ҷ�ֵ����һ���Ҷ�ֵ
		std::uint8_t gray = *img_col;
		std::uint8_t gray_last = *img_col;

		// �Խ���·���ϵ���һ�����أ��м���width+1������
		// ����Ҫ��һ���߽紦��
		// �ضԽ���ǰ����ʱ�������Ӱ���б߽磬�������кż�����ԭ����ǰ�����кŵ�������һ�߽�
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == width - 1 && current_row < height - 1) {
			// ����->���£����ұ߽�
			cost_init_col = cost_init + (current_row + direction) * width * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
		}
		else if (!is_forward && current_col == 0 && current_row > 0) {
			// ����->���ϣ�����߽�
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
		}
		else {
			cost_init_col += direction * (width + 1) * disp_range;
			cost_aggr_col += direction * (width + 1) * disp_range;
			img_col += direction * (width + 1);
		}

		// ·�����ϸ����ص���С����ֵ
		std::uint8_t mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// �Է����ϵ�2�����ؿ�ʼ��˳��ۺ�
		for (std::int32_t i = 0; i < height - 1; i ++) {
			gray = *img_col;
			std::uint8_t min_cost = UINT8_MAX;
			for (std::int32_t d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const std::uint8_t  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const std::uint8_t cost_s = cost + static_cast<std::uint8_t>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// �����ϸ����ص���С����ֵ�ʹ�������
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));

			// ��ǰ���ص����к�
			current_row += direction;
			current_col += direction;
			
			// ��һ������,����Ҫ��һ���߽紦��
			// ����Ҫ��һ���߽紦��
			// �ضԽ���ǰ����ʱ�������Ӱ���б߽磬�������кż�����ԭ����ǰ�����кŵ�������һ�߽�
			if (is_forward && current_col == width - 1 && current_row < height - 1) {
				// ����->���£����ұ߽�
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
			}
			else if (!is_forward && current_col == 0 && current_row > 0) {
				// ����->���ϣ�����߽�
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
			}
			else {
				cost_init_col += direction * (width + 1) * disp_range;
				cost_aggr_col += direction * (width + 1) * disp_range;
				img_col += direction * (width + 1);
			}

			// ����ֵ���¸�ֵ
			gray_last = gray;
		}
	}
}

void CostAggregateDagonal_2(const std::uint8_t* img_data, const std::int32_t& width, const std::int32_t& height,
	                        const std::int32_t& min_disparity, const std::int32_t& max_disparity, 
                            const std::int32_t& p1, const std::int32_t& p2_init,
	                        const std::uint8_t* cost_init, std::uint8_t* cost_aggr, bool is_forward) {
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// �ӲΧ
	const std::int32_t disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// ����(����->����) ��is_forward = true ; direction = 1
	// ����(����->����) ��is_forward = false; direction = -1;
	const std::int32_t direction = is_forward ? 1 : -1;

	// �ۺ�

	// �洢��ǰ�����кţ��ж��Ƿ񵽴�Ӱ��߽�
	std::int32_t current_row = 0;
	std::int32_t current_col = 0;

	for (std::int32_t j = 0; j < width; j++) {
		// ·��ͷΪÿһ�е���(β,dir=-1)������
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// ·�����ϸ����صĴ������飬������Ԫ����Ϊ�˱���߽��������β����һ����
		std::vector<std::uint8_t> cost_last_path(disp_range + 2, UINT8_MAX);

		// ��ʼ������һ�����صľۺϴ���ֵ���ڳ�ʼ����ֵ
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(std::uint8_t));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));

		// ·���ϵ�ǰ�Ҷ�ֵ����һ���Ҷ�ֵ
		std::uint8_t gray = *img_col;
		std::uint8_t gray_last = *img_col;

		// �Խ���·���ϵ���һ�����أ��м���width-1������
		// ����Ҫ��һ���߽紦��
		// �ضԽ���ǰ����ʱ�������Ӱ���б߽磬�������кż�����ԭ����ǰ�����кŵ�������һ�߽�
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == 0 && current_row < height - 1) {
			// ����->���£�����߽�
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
		}
		else if (!is_forward && current_col == width - 1 && current_row > 0) {
			// ����->���ϣ����ұ߽�
			cost_init_col = cost_init + (current_row + direction) * width * disp_range ;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
		}
		else {
			cost_init_col += direction * (width - 1) * disp_range;
			cost_aggr_col += direction * (width - 1) * disp_range;
			img_col += direction * (width - 1);
		}

		// ·�����ϸ����ص���С����ֵ
		std::uint8_t mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// ��·���ϵ�2�����ؿ�ʼ��˳��ۺ�
		for (std::int32_t i = 0; i < height - 1; i++) {
			gray = *img_col;
			std::uint8_t min_cost = UINT8_MAX;
			for (std::int32_t d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const std::uint8_t  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const std::uint8_t cost_s = cost + static_cast<std::uint8_t>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// �����ϸ����ص���С����ֵ�ʹ�������
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(std::uint8_t));

			// ��ǰ���ص����к�
			current_row += direction;
			current_col -= direction;

			// ��һ������,����Ҫ��һ���߽紦��
			// ����Ҫ��һ���߽紦��
			// �ضԽ���ǰ����ʱ�������Ӱ���б߽磬�������кż�����ԭ����ǰ�����кŵ�������һ�߽�
			if (is_forward && current_col == 0 && current_row < height - 1) {
				// ����->���£�����߽�
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
			}
			else if (!is_forward && current_col == width - 1 && current_row > 0) {
				// ����->���ϣ����ұ߽�
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
			}
			else {
				cost_init_col += direction * (width - 1) * disp_range;
				cost_aggr_col += direction * (width - 1) * disp_range;
				img_col += direction * (width - 1);
			}

			// ����ֵ���¸�ֵ
			gray_last = gray;
		}
	}
}

void MedianFilter(const float* in, float* out, 
                  const std::int32_t& width, const std::int32_t& height,
	              const std::int32_t wnd_size) {
	const std::int32_t radius = wnd_size / 2;
	const std::int32_t size = wnd_size * wnd_size;

	// �洢�ֲ������ڵ�����
	std::vector<float> wnd_data;
	wnd_data.reserve(size);

	for (std::int32_t i = 0; i < height; i++) {
		for (std::int32_t j = 0; j < width; j++) {
			wnd_data.clear();

			// ��ȡ�ֲ���������
			for (std::int32_t r = -radius; r <= radius; r++) {
				for (std::int32_t c = -radius; c <= radius; c++) {
					const std::int32_t row = i + r;
					const std::int32_t col = j + c;
					if (row >= 0 && row < height && col >= 0 && col < width) {
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}

			// ����
			std::sort(wnd_data.begin(), wnd_data.end());
			// ȡ��ֵ
			out[i * width + j] = wnd_data[wnd_data.size() / 2];
		}
	}
}

void RemoveSpeckles(float* disparity_map, const std::int32_t& width, const std::int32_t& height,
	                const std::int32_t& diff_insame, const std::uint32_t& min_speckle_aera, const float& invalid_val) {
	assert(width > 0 && height > 0);
	if (width < 0 || height < 0) {
		return;
	}

	// �����������Ƿ���ʵ�����
	std::vector<bool> visited(std::uint32_t(width*height),false);
	for(std::int32_t i=0;i<height;i++) {
		for(std::int32_t j=0;j<width;j++) {
			if (visited[i * width + j] || disparity_map[i*width+j] == invalid_val) {
				// �����ѷ��ʵ����ؼ���Ч����
				continue;
			}
			// ������ȱ������������
			// ����ͨ�����С����ֵ�������Ӳ�ȫ��Ϊ��Чֵ
			std::vector<std::pair<std::int32_t, std::int32_t>> vec;
			vec.emplace_back(i, j);
			visited[i * width + j] = true;
			std::uint32_t cur = 0;
			std::uint32_t next = 0;
			do {
				// ������ȱ����������	
				next = vec.size();
				for (std::uint32_t k = cur; k < next; k++) {
					const auto& pixel = vec[k];
					const std::int32_t row = pixel.first;
					const std::int32_t col = pixel.second;
					const auto& disp_base = disparity_map[row * width + col];
					// 8�������
					for(int r=-1;r<=1;r++) {
						for(int c=-1;c<=1;c++) {
							if(r==0&&c==0) {
								continue;
							}
							int rowr = row + r;
							int colc = col + c;
							if (rowr >= 0 && rowr < height && colc >= 0 && colc < width) {
								if(!visited[rowr * width + colc] &&
									(disparity_map[i * width + j] != invalid_val) &&
									abs(disparity_map[rowr * width + colc] - disp_base) <= diff_insame) {
									vec.emplace_back(rowr, colc);
									visited[rowr * width + colc] = true;
								}
							}
						}
					}
				}
				cur = next;
			} while (next < vec.size());

			// ����ͨ�����С����ֵ�������Ӳ�ȫ��Ϊ��Чֵ
			if(vec.size() < min_speckle_aera) {
				for(auto& pix:vec) {
					disparity_map[pix.first * width + pix.second] = invalid_val;
				}
			}
		}
	}
}

}     // namespace sgm_util
