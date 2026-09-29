#include "ClearanceModel.h"

#include <cmath>
#include <iostream>

namespace
{
Path make(std::initializer_list<int> locations)
{
    Path path;
    for (int location : locations)
        path.emplace_back(location);
    return path;
}

bool expect(bool condition, const char* message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << std::endl;
    return condition;
}
}

int main()
{
    Instance instance;
    instance.num_of_cols = 10;
    ClearanceModel model(clearance_mode::FIXED, 1.0, 1.0, 0.0);
    ClearanceConflictData conflict;
    bool ok = true;

    ok &= expect(!model.findFirst(instance, make({0, 1}), make({10, 11}), 0, 1, conflict),
                 "parallel paths outside clearance");
    ok &= expect(model.findFirst(instance, make({0, 11}), make({1, 10}), 0, 1, conflict),
                 "diagonal crossing must be unsafe");
    ok &= expect(conflict.tau > 0.0 && conflict.tau < 1.0, "crossing conflict is swept");
    ClearanceModel wait_model(clearance_mode::FIXED, 1.0, 1.1, 0.0);
    ok &= expect(wait_model.findFirst(instance, make({0, 0}), make({1, 1}), 0, 1, conflict),
                 "wait while another robot passes nearby");

    ClearanceModel equality(clearance_mode::FIXED, 1.0, 2.0, 0.0);
    ok &= expect(!equality.findFirst(instance, make({0}), make({2}), 0, 1, conflict),
                 "exactly equal clearance is safe");

    return ok ? 0 : 1;
}
