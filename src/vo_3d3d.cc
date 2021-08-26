#include "io_utils.h"
#include "math_utils.h"
#include "evaluate.h"

int main(int argc, char **argv) 
{
    std::cout << std::endl;
    std::cout << "3D-3D VISUAL ODOMETRY" << std::endl;
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
    std::vector<Eigen::Vector3d> left_prev_3d;

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

        // Compute 3D points for current frame

        std::vector<Eigen::Vector3d> left_cur_3d;
        left_cur_3d.resize(left_cur_2d.size());
        
        for (int j = 0; j < left_cur_2d.size(); ++j)
        {
            if(match_LR_cur[j] != NOMATCH)
            {
                double d = left_cur_2d[j].x() - right_cur_2d[match_LR_cur[j]].x();
                if(d > params.min_disparity)
                {
                    double stereopoint[3] = { left_cur_2d[j].x(), 
                                              right_cur_2d[match_LR_cur[j]].x(), 
                                              (left_cur_2d[j].y() + right_cur_2d[match_LR_cur[j]].y()) / 2 };

                    camera.ProjectPoint(&left_cur_3d[j].x(), stereopoint);
                }
                else
                {
                    left_cur_3d[j].z() = 0.;
                }
            }
            else
            {
                left_cur_3d[j].z() = 0.;
            }
        }

        if(i > 0)
        {
            // Compute centroids

            Eigen::Vector3d centroid_prev, centroid_cur;
            centroid_prev.setZero();
            centroid_cur.setZero();

            int n_pts = 0;

            for (int j = 0; j < left_prev_3d.size(); j++)
            {
                if (left_prev_3d[j].z() > 0 && match_prev_cur[j] != NOMATCH && left_cur_3d[match_prev_cur[j]].z() > 0)
                {
                    centroid_prev = (centroid_prev + left_prev_3d[j]).eval();
                    centroid_cur = (centroid_cur + left_cur_3d[match_prev_cur[j]]).eval();
                    n_pts++;
                }
            }

            centroid_prev = (centroid_prev / n_pts).eval();
            centroid_cur = (centroid_cur / n_pts).eval();
           
            // Compute correlation matrix

            Eigen::Matrix3d H;
            H.setZero();

            for (int j = 0; j < left_prev_3d.size(); j++)
            {
                if (left_prev_3d[j].z() > 0 && match_prev_cur[j] != NOMATCH && left_cur_3d[match_prev_cur[j]].z() > 0)
                {
                    H = (H + (left_cur_3d[match_prev_cur[j]] - centroid_cur) * (left_prev_3d[j] - centroid_prev).transpose()).eval();
                }
            }
            
            // Factor H = (R,t)
            
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R = svd.matrixV() * svd.matrixU().transpose();
            t = centroid_prev - R * centroid_cur;
        }

        if (i > 0)
        {     
            t0 = (Ri0 * t + t0).eval();
            Ri0 = (Ri0 * R).eval();
        }

        SaveCurrentPose(out, Ri0,  t0);

        left_prev_2d = left_cur_2d;
        left_prev_3d = left_cur_3d;
    }

	out.close();

	if (!EvaluateResults(params.gt_seqname, params.seqname))
    {
        std::cout << "Failed to evaluate results" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
