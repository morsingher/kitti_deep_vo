#include "math_utils.h"

int CountOptimizationVariables(const std::vector<VOPoint>& pts)
{
    int n = 0;

    for(const auto& p : pts)
    {
        if (p.use_in_opt)
        {
            n++;
        }
    }

    return n;
}