#ifndef VO_IO_UTILS_H
#define VO_IO_UTILS_H

#include "common.h"

class Parameters
{
public:

	std::string project_path;
	std::string sequence_path;
	std::string seqname;
	std::string gt_seqname;

	int num_frames;

	double focal;
	double center_x;
	double center_y;
	double baseline;

	double min_disparity;

	int max_IRLS_iter;
	int th_inliers;
	int th_outliers;

	double integration_th;
	int max_age_th;

	bool Load(const char* filename);
};

bool ReadPointFile(std::vector<Eigen::Vector2d>& v, const std::string& filename);
bool ReadMatchFile(std::vector<int>& v, const std::string& filename);
int CountValidMatches(const std::vector<int>& matches);
void SaveCurrentPose(std::ofstream& out, const Eigen::Matrix3d& Ri0, const Eigen::Vector3d& t0);

#endif