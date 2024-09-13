#pragma once
#include "bonxai/bonxai.hpp"
#include <string>
#include <Eigen/Eigen>

namespace Bonxai
{

bool ReadPointsFromPCD(const std::string& filepath,
                       std::vector<Eigen::Vector3d>& points);

bool ReadPointsFromPCD(const std::string& filepath, std::vector<Point3D>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Eigen::Vector3d>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Bonxai::Point3D>& points);

void WritePointsFromPCD(const std::string& filepath,
                        const std::vector<Bonxai::CoordT>& points);

}  // namespace Bonxai
