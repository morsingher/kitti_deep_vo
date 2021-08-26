#include "io_utils.h"

bool Parameters::Load(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == nullptr)
    {
        std::cout << "Couldn't open the configuration file." << std::endl;
        return false;
    }

    char read_buffer[65536];
    rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer)); 
    rapidjson::Document d;
    d.ParseStream(is);
    fclose(fp);

    project_path = d["project_path"].GetString();
    sequence_path = d["sequence_path"].GetString();
    seqname = sequence_path + d["seqname"].GetString();
    gt_seqname = sequence_path + d["gt_seqname"].GetString();

    num_frames = d["num_frames"].GetInt();

    focal = d["focal"].GetDouble();
    center_x = d["center_x"].GetDouble();
    center_y = d["center_y"].GetDouble();
    baseline = d["baseline"].GetDouble();

    min_disparity = d["min_disparity"].GetDouble();

    max_IRLS_iter = d["max_IRLS_iter"].GetInt();
    th_inliers = d["th_inliers"].GetInt();
    th_outliers = d["th_outliers"].GetInt();

    integration_th = d["integration_th"].GetDouble();
    max_age_th = d["max_age_th"].GetInt();

    return true;
}

bool ReadPointFile(std::vector<Eigen::Vector2d>& v, const std::string& filename)
{
    std::ifstream fs(filename, std::ios::in);
    if(!fs)
    {
        return false;
    }
    
    while(fs)
    {
        Eigen::Vector2d p;
        fs >> p.x() >> p.y();
        if(fs)
        {
            v.push_back(p);
        }
    }

    return true;
}

bool ReadMatchFile(std::vector<int>& v, const std::string& filename)
{
    std::ifstream fs(filename, std::ios::in);
    if(!fs)
    {
        return false;
    }
    
    while(fs)
    {
        int m;
        fs >> m;
        if(fs)
        {
            v.push_back(m);
        }
    }

    return true;
}

int CountValidMatches(const std::vector<int>& matches)
{
    int n = 0;

    for (const auto& id : matches)
    {
        if (id != NOMATCH)
        {
            n++;
        }
    }
    
    return n;
}

void SaveCurrentPose(std::ofstream& out, const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    out << R(0,0) << ' ' << R(0,1) << ' ' << R(0,2) << ' ' << t[0] << ' '
        << R(1,0) << ' ' << R(1,1) << ' ' << R(1,2) << ' ' << t[1] << ' '
        << R(2,0) << ' ' << R(2,1) << ' ' << R(2,2) << ' ' << t[2] << std::endl;
}