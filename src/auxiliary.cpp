//
// Created by tzuk on 4/19/23.
//

#include "auxiliary.h"

namespace LiteSimulator
{
    std::vector<Eigen::Vector3d> LiteSimulator::auxiliary::readCsv(std::string &csvPath, char delimiter)
    {
        std::ifstream pointData(csvPath);
        std::vector<Eigen::Vector3d> points;
        while (!pointData.eof())
        {
            std::vector<std::string> row;
            std::string line;
            std::string word;
            std::getline(pointData, line);

            std::stringstream words(line);

            if (line.empty())
            {
                continue;
            }

            while (std::getline(words, word, delimiter))
            {
                try
                {
                    std::stod(word);
                }
                catch (std::out_of_range &e)
                {
                    word = "0";
                }
                row.push_back(word);
            }
            points.emplace_back(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
        }
        return points;
    }

    std::vector<Eigen::Vector3d>
    auxiliary::filterPointsByPose(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &cameraPosition,
                                  Eigen::Vector3d &cameraAngles)
    {
        Eigen::Matrix3d Rcw = (Eigen::AngleAxisd(-cameraAngles[0], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(-cameraAngles[2], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(-cameraAngles[1], Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix();
        Eigen::Matrix3d Rwc = Rcw.transpose();
        Eigen::Vector3d tcw(-cameraPosition[0], cameraPosition[1], -cameraPosition[2]);
        std::vector<Eigen::Vector3d> filteredPoints;
            return {};
    }
}
