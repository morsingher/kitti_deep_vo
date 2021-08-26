#ifndef VO_MATH_UTILS_H
#define VO_MATH_UTILS_H

#include "common.h"

struct StereoPoint 
{
    double ul; // Left column
    double ur; // Right column
    double v;  // Common row
};

class StereoCamera 
{
public:

    double f; // Focal length
    double u0, v0; // Principal point
    double b; // Baseline

    StereoCamera(double _f, double _u0, double _v0, double _b) : f(_f), u0(_u0), v0(_v0), b(_b) {}

    // Convert a 3D camera point (x,y,z) in a StereoPoint (ul, ur, v)

    template<class T>
    void ObservePoint(T *out, const T *pt) const
    {
        out[0] = T(f) * pt[0] / pt[2] + T(u0);          // ul
        out[1] = T(f) * (pt[0] - T(b)) / pt[2] + T(u0); // ur
        out[2] = T(f) * (pt[1]) / pt[2] + T(v0);        // v
    }

    // Convert a StereoPoint (ul, ur, v) in a 3D camera point (x,y,z)
    
    template<class T>
    void ProjectPoint(T *out, const T *pt) const
    {
        T d = pt[0] - pt[1];
        out[0] = (pt[0] - u0) * b / d; // x
        out[1] = (pt[2] - v0) * b / d; // y
        out[2] = f * b / d;            // z
    }
};

class VOPoint 
{
public:

    bool valid; // True if prev and cur are valid (z > 0)
    bool use_in_opt; // Use this point for optimization

    double w; // Weight for robust estimator
    int age; // Tracked age
    
    StereoPoint prev; // Previous point (ul, ur, v)
    StereoPoint cur; // Current point (ul, ur, v)

    VOPoint() : valid(false), use_in_opt(false), w(1.0), age(0) {}

    // Predict point according to vo estimate

    template<class T>
    void predict(const StereoCamera& camera, const T* vo, T* cur) 
    {
        T pprev2d[3]; // Previous 2D point
        T pprev3d[3]; // Previous 3D point
        T pcur3d[3]; // Predicted 3D point

        pprev2d[0] = T(prev.ul);
        pprev2d[1] = T(prev.ur);
        pprev2d[2] = T(prev.v);

        // Project previous 2D point into 3D

        camera.ProjectPoint(pprev3d, pprev2d);

        // Transform pprev3d according to vo

        T angle_axis[3];
        angle_axis[0] = vo[0];
        angle_axis[1] = vo[1];
        angle_axis[2] = vo[2];

        ceres::AngleAxisRotatePoint(angle_axis, pprev3d, pcur3d);

        pcur3d[0] += vo[3];
        pcur3d[1] += vo[4];
        pcur3d[2] += vo[5];

        // Observe cur from pcur3d

        camera.ObservePoint(cur, pcur3d);
    }


    // Compute residual between current and observed point according to vo estimate

    template<class T>
    void residuals(const StereoCamera& camera, const T* vo, T* res)
    {
        T pcur[3];
        predict(camera, vo, pcur);
        res[0] = T(cur.ul) - pcur[0];
        res[1] = T(cur.ur) - pcur[1];
        res[2] = T(cur.v)  - pcur[2];
    }
};

// Custom wrapper for Ceres optimization

class VO2D3DCostFunction {

    std::vector<VOPoint> m_pts;
    StereoCamera camera;

public:

    VO2D3DCostFunction(const std::vector<VOPoint>& _pts, const StereoCamera& _camera) : m_pts(_pts), camera(_camera) {}

    template<class T>
    bool operator()(const T* const * parameters, T * residuals)
    {
        const T *p = parameters[0];
        int j = 0;
        for (int i = 0; i<m_pts.size(); ++i)
        {
            if (m_pts[i].use_in_opt)
            {
                T out[3];
                m_pts[i].residuals(camera, p, out);
                residuals[3*j+0] = T(m_pts[i].w) * out[0];
                residuals[3*j+1] = T(m_pts[i].w) * out[1];
                residuals[3*j+2] = T(2 * m_pts[i].w) * out[2];
                ++j;
            }
        }
        return true;
    }
};

int CountOptimizationVariables(const std::vector<VOPoint>& pts);

#endif