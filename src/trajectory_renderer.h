#pragma once

#include <vector>

namespace RE
{
    class NiCamera;
    class NiPoint3;
}

namespace TrajectoryRenderer
{
    bool Install();
    void Submit(const std::vector<RE::NiPoint3>& path, RE::NiCamera* camera);
    void Clear();
}
