#include "ClearanceModel.h"

#include <algorithm>

namespace
{
PathEntry at(const Path& path, int timestep)
{
    return path.at(std::min<int>(timestep, static_cast<int>(path.size()) - 1));
}

double distance2(double ax, double ay, double bx, double by)
{
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}
}

bool ClearanceModel::findFirst(const Instance& instance, const Path& path1, const Path& path2,
                               int a1, int a2, ClearanceConflictData& result) const
{
    if (!enabled() || path1.empty() || path2.empty())
        return false;
    const double required = requiredClearance(a1, a2);
    if (required < 0.0)
        return false;
    const double required2 = required * required;
    const int horizon = std::max(path1.size(), path2.size());
    constexpr double eps = 1e-10;

    // Check vertex proximity first. Ordinary same-vertex conflicts are left to
    // the stock detector, so this method only reports distinct nearby cells.
    for (int t = 0; t < horizon; ++t)
    {
        const int l1 = at(path1, t).location;
        const int l2 = at(path2, t).location;
        if (l1 == l2)
            continue;
        const auto p1 = metricPosition(instance, l1);
        const auto p2 = metricPosition(instance, l2);
        const double d2 = distance2(p1.first, p1.second, p2.first, p2.second);
        if (d2 + eps < required2)
        {
            result = {};
            result.timestep = t;
            result.a1_from = l1;
            result.a1_to = l1;
            result.a2_from = l2;
            result.a2_to = l2;
            result.tau = 0.0;
            result.min_distance = std::sqrt(d2);
            result.required_clearance = required;
            result.vertex_start = true;
            result.vertex_end = true;
            return true;
        }
    }

    // Check each simultaneous swept transition. If a minimum occurs at an
    // endpoint it was already covered by the vertex loop above.
    for (int t = 0; t + 1 < horizon; ++t)
    {
        const int a0 = at(path1, t).location;
        const int a1loc = at(path1, t + 1).location;
        const int b0 = at(path2, t).location;
        const int b1loc = at(path2, t + 1).location;
        if (a0 == b0 || (a0 == b1loc && b0 == a1loc))
            continue; // stock vertex/edge conflict owns these cases
        const auto pa0 = metricPosition(instance, a0);
        const auto pa1 = metricPosition(instance, a1loc);
        const auto pb0 = metricPosition(instance, b0);
        const auto pb1 = metricPosition(instance, b1loc);
        const double qx = pa0.first - pb0.first;
        const double qy = pa0.second - pb0.second;
        const double vx = (pa1.first - pa0.first) - (pb1.first - pb0.first);
        const double vy = (pa1.second - pa0.second) - (pb1.second - pb0.second);
        const double vv = vx * vx + vy * vy;
        double tau = vv > eps ? -(qx * vx + qy * vy) / vv : 0.0;
        tau = std::max(0.0, std::min(1.0, tau));
        const double dx = qx + tau * vx;
        const double dy = qy + tau * vy;
        const double d2 = dx * dx + dy * dy;
        if (d2 + eps < required2 && tau > eps && tau < 1.0 - eps)
        {
            result = {};
            result.timestep = t;
            result.a1_from = a0;
            result.a1_to = a1loc;
            result.a2_from = b0;
            result.a2_to = b1loc;
            result.tau = tau;
            result.min_distance = std::sqrt(d2);
            result.required_clearance = required;
            return true;
        }
    }
    return false;
}

bool ClearanceModel::validate(const Instance& instance, const vector<Path>& paths,
                              double* minimum_distance) const
{
    double minimum = std::numeric_limits<double>::infinity();
    if (!enabled())
        return true;
    for (int i = 0; i < static_cast<int>(paths.size()); ++i)
    {
        for (int j = i + 1; j < static_cast<int>(paths.size()); ++j)
        {
            ClearanceConflictData conflict;
            if (findFirst(instance, paths[i], paths[j], i, j, conflict))
                return false;
            const int horizon = std::max(paths[i].size(), paths[j].size());
            for (int t = 0; t < horizon; ++t)
            {
                const auto p1 = metricPosition(instance, at(paths[i], t).location);
                const auto p2 = metricPosition(instance, at(paths[j], t).location);
                minimum = std::min(minimum, std::sqrt(distance2(p1.first, p1.second, p2.first, p2.second)));
            }
        }
    }
    if (minimum_distance != nullptr)
        *minimum_distance = minimum;
    return true;
}

