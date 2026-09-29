#pragma once

#include "common.h"
#include "Instance.h"

#include <cmath>
#include <limits>

enum class clearance_mode { POINT, FIXED, RADII };

struct ClearanceConflictData
{
    int timestep = 0;
    int a1_from = -1;
    int a1_to = -1;
    int a2_from = -1;
    int a2_to = -1;
    double tau = 0.0;
    double min_distance = std::numeric_limits<double>::infinity();
    double required_clearance = 0.0;
    bool vertex_start = false;
    bool vertex_end = false;
};

// Metric, synchronized swept-distance policy used by the clearance-aware fork.
// Locations are EECBS linearized row/column cells and are converted to metres
// using grid_resolution. The point mode deliberately preserves upstream
// vertex/edge-only behavior.
class ClearanceModel
{
public:
    static constexpr const char* capability = "clearance-aware-v1";

    ClearanceModel() = default;
    ClearanceModel(clearance_mode mode, double grid_resolution,
                   double planning_clearance, double margin,
                   const vector<double>& radii = {})
        : mode_(mode), grid_resolution_(grid_resolution),
          planning_clearance_(planning_clearance), margin_(margin), radii_(radii) {}

    void configure(clearance_mode mode, double grid_resolution,
                   double planning_clearance, double margin,
                   const vector<double>& radii = {})
    {
        mode_ = mode;
        grid_resolution_ = grid_resolution;
        planning_clearance_ = planning_clearance;
        margin_ = margin;
        radii_ = radii;
    }

    clearance_mode mode() const { return mode_; }
    double gridResolution() const { return grid_resolution_; }
    double planningClearance() const { return planning_clearance_; }
    double margin() const { return margin_; }
    bool enabled() const { return mode_ != clearance_mode::POINT; }

    double requiredClearance(int a1, int a2) const
    {
        if (mode_ == clearance_mode::POINT)
            return 0.0;
        if (mode_ == clearance_mode::RADII)
        {
            if (a1 < 0 || a2 < 0 || a1 >= static_cast<int>(radii_.size()) ||
                a2 >= static_cast<int>(radii_.size()))
                return -1.0;
            return radii_[a1] + radii_[a2] + margin_;
        }
        return planning_clearance_;
    }

    pair<double, double> metricPosition(const Instance& instance, int location) const
    {
        const auto row_col = instance.getCoordinate(location);
        return make_pair(static_cast<double>(row_col.second) * grid_resolution_,
                         static_cast<double>(row_col.first) * grid_resolution_);
    }

    // Return the first strictly unsafe metric conflict for a pair of paths.
    // Completed paths are held at their goals. Endpoint conflicts are reported
    // as vertex conflicts; only an interior minimum uses directed edge splits.
    bool findFirst(const Instance& instance, const Path& path1, const Path& path2,
                   int a1, int a2, ClearanceConflictData& result) const;

    bool validate(const Instance& instance, const vector<Path>& paths,
                  double* minimum_distance = nullptr) const;

private:
    clearance_mode mode_ = clearance_mode::POINT;
    double grid_resolution_ = 1.0;
    double planning_clearance_ = 0.0;
    double margin_ = 0.0;
    vector<double> radii_;
};

