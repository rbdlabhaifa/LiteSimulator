//
// Created by tzuk on 4/19/23.
//

#ifndef LiteSimulator_SIMULATOR_H
#define LiteSimulator_SIMULATOR_H
#include <string>
#include <vector>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/geometry/glgeometry.h>
#include "../utils/TextureShader.h"
namespace LiteSimulator {

    class simulator {
        
        public:
        void run(std::string &cloudPointsPath,std::string &modelPath,Eigen::Vector3d &startPosition,Eigen::VectorXd degreesOfFreedomOffsets,double frameWidth,double frameHeight,double fx,double fy,double cx,double cy,double scaleFactor);
        void drawPoints(std::vector<Eigen::Vector3d> &points,Eigen::Vector3d &offsets,float scaleFactor);
    };

} // LiteSimulator

#endif //LiteSimulator_SIMULATOR_H
