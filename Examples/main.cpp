#include <iostream>
#include "simulator.h"
int main()
{
    std::string dataLocation = "../data/cloud1.csv";
        std::string ModelLocation = "/home/tzuk/Documents/TLV_lab/FBX/drones_lab.obj";

    Eigen::RowVectorXd offsets(6);
    offsets << 0.0696954, 0.232057, 2.83821, 0.004893,-0.35643,-0.0139628;
    Eigen::Vector3d startPos(1.0, 1.0, 1.0);
    LiteSimulator::simulator simulator;
    simulator.run(dataLocation,ModelLocation,startPos, offsets,640,480,619.6508392029048,618.5264705043031,321.88905699582324,243.8086797913814, 0.0305);
}
