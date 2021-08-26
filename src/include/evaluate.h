#ifndef VO_EVALUATE_H
#define VO_EVALUATE_H

#include "matrix.h"
#include "common.h"

struct Error 
{
  double r_err, t_err;
};

bool LoadPoses(std::vector<Matrix>& poses, const std::string& filename);
std::vector<double> ComputeTrajectoryDistances(const std::vector<Matrix>& poses);
int ComputeLastFrame(const std::vector<double>& dist, const int first_frame, const double len);
double ComputeRotationError(const Matrix& pose_error);
double ComputeTranslationError(const Matrix& pose_error);
std::vector<Error> ComputeErrors (const std::vector<Matrix>& poses_gt, const std::vector<Matrix>& poses_est);
void PrintErrors (const std::vector<Error>& err);
bool EvaluateResults(const std::string& path_gt, const std::string& path_est);

#endif
