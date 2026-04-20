#include "RobotDefinition.hpp"
#include "Interface/MuJoCo/MuJoCoSimulationBridge.hpp"

int main()
{
    RobotDefinition robot;
    robot.model_xml = CMAKE_SOURCE_DIR"/model/scene.xml";

    MuJoCoSimulationBridge sim(robot, robot.model_xml);
    sim.run();
    return 0;
}
