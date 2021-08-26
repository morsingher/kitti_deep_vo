#include "io_utils.h"
#include "math_utils.h"
#include "evaluate.h"

int main(int argc, char **argv) 
{
    std::cout << std::endl;
    std::cout << "3D-2D VISUAL ODOMETRY" << std::endl;
    std::cout << std::endl;

    // Check if the configuration file has been provided

    if (argc != 2)
    {
        std::cout << "Usage " << argv[0] << " PATH_TO_CONFIG_FILE" << std::endl;
        return EXIT_FAILURE;
    }

    // Load parameters

    std::cout << "Loading parameters from: " << std::string(argv[1]) << std::endl;
    Parameters params;
    if (!params.Load(argv[1]))
    {
        std::cout << "Failed to load parameters!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Done!" << std::endl << std::endl;

    std::ofstream out(params.seqname, std::ios::out);
    StereoCamera camera(params.focal, params.center_x, params.center_y, params.baseline);

    // VO algorithm

    Eigen::Matrix3d Ri0;
    Eigen::Vector3d t0;
    t0.setZero();
    Ri0.setIdentity();
    
    std::vector<Eigen::Vector2d> left_prev_2d;
    std::vector<VOPoint> ptsprev;
    
    double vo[6]= {0,0,0,0,0,-1.}; // Initial guess

    for (int i = 0; i < params.num_frames; ++i)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        R.setIdentity();
        t.setZero();
        
        std::stringstream ss;

        // Load left keypoints for current frame

        std::vector<Eigen::Vector2d> left_cur_2d;
        ss << params.sequence_path << "kp_0_" << std::setw(6) << std::setfill('0') << i << ".txt";
        if (!ReadPointFile(left_cur_2d, ss.str()))
        {
            std::cout << "Failed to open file: " << ss.str() << std::endl;
            return EXIT_FAILURE;
        }
        ss.str(std::string());

        // Load right keypoints for current frame

        std::vector<Eigen::Vector2d> right_cur_2d;
        ss << params.sequence_path << "kp_1_" << std::setw(6) << std::setfill('0') << i << ".txt";
        if (!ReadPointFile(right_cur_2d, ss.str()))
        {
            std::cout << "Failed to open file: " << ss.str() << std::endl;
            return EXIT_FAILURE;
        }
        ss.str(std::string());

        // Load matches between left and right keypoints

        std::vector<int> match_LR_cur;
        ss << params.sequence_path << "stereo_" << std::setw(6) << std::setfill('0') << i << ".txt";
        if(!ReadMatchFile(match_LR_cur, ss.str()))
        {
            std::cout << "Failed to open file: " << ss.str() << std::endl;
            return EXIT_FAILURE;
        }
        ss.str(std::string());

        // Load matches between previous and current frame

        std::vector<int> match_prev_cur;
        if(i > 0)
        {
            ss << params.sequence_path << "flow_" << std::setw(6) << std::setfill('0') << i << ".txt";
            if(!ReadMatchFile(match_prev_cur, ss.str()))
            {
                std::cout << "Failed to open file: " << ss.str() << std::endl;
                return EXIT_FAILURE;
            }
        }

        std::cout << "Frame " << i << "/" << params.num_frames << " | " << left_cur_2d.size() << " " << right_cur_2d.size(); 
        std::cout << " " << CountValidMatches(match_LR_cur) << " " << CountValidMatches(match_prev_cur) << std::endl;

        // Compute VO points

        std::vector<VOPoint> pts;
        pts.resize(left_cur_2d.size());
        
        for(int j = 0; j < left_cur_2d.size(); ++j)
        {
            if(match_LR_cur[j] != NOMATCH)
            {
                double d = left_cur_2d[j].x() - right_cur_2d[match_LR_cur[j]].x();
                if(d > params.min_disparity)
                {
                    pts[j].valid = true;         
                    pts[j].cur.ul = left_cur_2d[j].x();
                    pts[j].cur.ur = right_cur_2d[match_LR_cur[j]].x();
                    pts[j].cur.v = (left_cur_2d[j].y() + right_cur_2d[match_LR_cur[j]].y()) / 2.;
                }
            }
        }

        if(i > 0)
        {
            for (int j = 0; j < ptsprev.size(); ++j)
            {
                if (ptsprev[j].valid && match_prev_cur[j] != NOMATCH && pts[match_prev_cur[j]].valid)
                {
                    pts[match_prev_cur[j]].use_in_opt = true;
                    pts[match_prev_cur[j]].prev = ptsprev[j].cur;
                    pts[match_prev_cur[j]].age = std::min(ptsprev[j].age + 1, params.max_age_th); // Task 5
                }
            }
            
            // Create optimization problem for Ceres and solve it (IRLS)

            const int max_iter = params.max_IRLS_iter; 

            for (int j = 0; j < max_iter; j++)
            {
                for (int k = 0; k < pts.size(); k++)
                {
                    if (pts[k].use_in_opt)
                    {
                        // Reject outliers

                        double res[3];
                        pts[k].residuals(camera, vo, res);
                        double r2 = std::pow(res[0], 2) + std::pow(res[1], 2) + std::pow(res[2], 2);
                        
                        if (j == 0 && r2 > params.th_outliers)
                        {
                            pts[k].w = 1e-6;
                        }
                        else
                        {
                            pts[k].w = pts[k].age * params.th_inliers / std::max(static_cast<double>(params.th_inliers), r2);
                        }
                    }
                }

                ceres::Problem problem;
                ceres::Solver::Options options;

                options.minimizer_progress_to_stdout = false;
                options.max_num_iterations = 10;
                VO2D3DCostFunction *fun = new VO2D3DCostFunction(pts, camera);

                ceres::DynamicAutoDiffCostFunction<VO2D3DCostFunction> *cost =
                    new ceres::DynamicAutoDiffCostFunction<VO2D3DCostFunction>(fun);

                cost->AddParameterBlock(6);
                cost->SetNumResiduals(3 * CountOptimizationVariables(pts));

                problem.AddResidualBlock(cost, 0, vo);

                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);
            }

            // Transform vo -> (R,t)

            double angle_axis[3];
            angle_axis[0] = vo[0];
            angle_axis[1] = vo[1];
            angle_axis[2] = vo[2];

            ceres::AngleAxisToRotationMatrix(angle_axis, ceres::ColumnMajorAdapter3x3((double*)R.data()));
            R.transposeInPlace();

            t = - R * Eigen::Vector3d(vo[3], vo[4], vo[5]);
        }

        for (int j = 0; j < pts.size(); j++)
        {
            if (pts[j].use_in_opt)
            {
                double pred[3];
                pts[j].predict(camera, vo, pred);

                double e_ul = pts[j].cur.ul - pred[0];
                double e_ur = pts[j].cur.ur - pred[1];
                double e_v = pts[j].cur.v - pred[2];

                double r2 = std::pow(e_ul, 2) + std::pow(e_ur, 2) + std::pow(e_v, 2);

                if (r2 < params.integration_th)
                {
                    pts[j].cur.ul = (pts[j].age * pred[0] + pts[j].cur.ul) / (pts[j].age + 1);
                    pts[j].cur.ur = (pts[j].age * pred[1] + pts[j].cur.ur) / (pts[j].age + 1);
                    pts[j].cur.v = (pts[j].age * pred[2] + pts[j].cur.v) / (pts[j].age + 1);
                }
                else
                {
                    pts[j].age = 1;
                }
            }
        }

        if (i > 0)
        {     
            t0 = (Ri0 * t + t0).eval();
            Ri0 = (Ri0 * R).eval();
        }

        SaveCurrentPose(out, Ri0, t0);

        left_prev_2d = left_cur_2d;
        ptsprev = pts;
    }

    out.close();

    if (!EvaluateResults(params.gt_seqname, params.seqname))
    {
        std::cout << "Failed to evaluate results" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
