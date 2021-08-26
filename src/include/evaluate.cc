#include "evaluate.h"

bool LoadPoses(std::vector<Matrix>& poses, const std::string& filename) 
{
  std::ifstream fs(filename);
  if (!fs)
  {
    std::cout << "Failed to open: " << filename << std::endl;
    return false;
  }

  while (fs)
  {
    Matrix p = Matrix::eye(4);
    fs >> p.val[0][0] >> p.val[0][1] >> p.val[0][2] >> p.val[0][3]
       >> p.val[1][0] >> p.val[1][1] >> p.val[1][2] >> p.val[1][3]
       >> p.val[2][0] >> p.val[2][1] >> p.val[2][2] >> p.val[2][3];

    if (fs)
    {
      poses.push_back(p);
    }
  }

  return true;
}

std::vector<double> ComputeTrajectoryDistances(const std::vector<Matrix>& poses) 
{
  std::vector<double> dist;
  dist.push_back(0.0);

  for (int i = 1; i < poses.size(); i++) 
  {
    const Matrix P1 = poses[i-1];
    const Matrix P2 = poses[i];
    const double dx = P1.val[0][3] - P2.val[0][3];
    const double dy = P1.val[1][3] - P2.val[1][3];
    const double dz = P1.val[2][3] - P2.val[2][3];
    dist.push_back(dist[i-1] + sqrt(dx * dx + dy * dy + dz * dz));
  }

  return dist;
}

int ComputeLastFrame(const std::vector<double>& dist, const int first_frame, const double len) 
{
  for (int i = first_frame; i < dist.size(); i++)
  {
    if (dist[i] > dist[first_frame] + len)
    {
      return i;
    }
  }
  return -1;
}

double ComputeRotationError(const Matrix& pose_error) 
{
  const double a = pose_error.val[0][0];
  const double b = pose_error.val[1][1];
  const double c = pose_error.val[2][2];
  const double d = 0.5 * (a + b + c - 1.0);
  return std::acos(std::max(std::min(d, 1.0), -1.0));
}

double ComputeTranslationError(const Matrix& pose_error) 
{
  const double dx = pose_error.val[0][3];
  const double dy = pose_error.val[1][3];
  const double dz = pose_error.val[2][3];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<Error> ComputeErrors (const std::vector<Matrix>& poses_gt, const std::vector<Matrix>& poses_est) 
{
  std::vector<Error> err;

  const int step_size = 10;
  const std::vector<double> lengths = { 100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0 };
  const int num_lengths = 8;
  
  const std::vector<double> dist = ComputeTrajectoryDistances(poses_gt);

  for (int first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) 
  {
    for (int i = 0; i < num_lengths; i++) 
    {
      const double len = lengths[i];
      const int last_frame = ComputeLastFrame(dist, first_frame, len);
      
      if (last_frame == -1 || last_frame >= poses_est.size())
      {
        continue;
      }

      Matrix pose_delta_gt = Matrix::inv(poses_gt[first_frame]) * poses_gt[last_frame];
      Matrix pose_delta_est = Matrix::inv(poses_est[first_frame]) * poses_est[last_frame];
      Matrix pose_error = Matrix::inv(pose_delta_est) * pose_delta_gt;
      double r_err = ComputeRotationError(pose_error);
      double t_err = ComputeTranslationError(pose_error);

      err.push_back({ r_err/len, t_err/len });
    }
  }

  return err;
}

void PrintErrors (const std::vector<Error>& err) 
{

  double t_err = 0;
  double r_err = 0;

  for (const auto& e : err) 
  {
    t_err += e.t_err;
    r_err += e.r_err;
  }

  double num = err.size();

  std::cout << std::endl << "Translation error: " << t_err / num << std::endl;
  std::cout << "Rotation error: " << r_err / num << std::endl << std::endl; 
}

bool EvaluateResults(const std::string& path_gt, const std::string& path_est) 
{
  std::vector<Matrix> poses_gt, poses_est;

  std::cout << std::endl << "Evaluating results..." << std::endl;
  std::cout << "Loading ground truth poses..." << std::endl;

  if (!LoadPoses(poses_gt, path_gt))
  {
    std::cout << "Failed to load ground truth poses from: " << path_gt << std::endl;
    return false;
  }

  std::cout << "Loading estimated poses..." << std::endl;

  if (!LoadPoses(poses_est, path_est))
  {
    std::cout << "Failed to load estimated poses from: " << path_est << std::endl;
    return false;
  }

  std::cout << "Computing errors..." << std::endl;

  std::vector<Error> errors = ComputeErrors(poses_gt, poses_est);

  PrintErrors(errors);

  return true;
}