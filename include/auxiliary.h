//
// Created by tzuk on 4/19/23.
//

#ifndef LiteSimulator_AUXILIARY_H
#define LiteSimulator_AUXILIARY_H

#include <vector>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

namespace LiteSimulator {
    class auxiliary {
        public:
       static std::vector<Eigen::Vector3d> readCsv(std::string &csvPath, char delimiter);
///
/// \param points
/// \param cameraPosition should be by right hand rule, x,z as plain and y as height
/// \param cameraAngles
/// \return
        std::vector<Eigen::Vector3d> filterPointsByPose(std::vector<Eigen::Vector3d> &points,Eigen::Vector3d &cameraPosition,Eigen::Vector3d &cameraAngles);
    };
}
#endif //LiteSimulator_AUXILIARY_H
