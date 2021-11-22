#include <Arduino.h>
#include "proj_types.h"
#include "robot.h"

void control(robot_t& robot)
{
    robot.tis = millis() - robot.tes;

    // Rules for the state evolution
     if (robot.state == 2 && robot.x > 0.2) {
      robot.setState(0);

    } else if (robot.state == 202 && robot.tis > robot.T1) {
      robot.setState(200);
    }

    // Actions in each state
    if (robot.state == 0) {
      robot.setRobotVW(0, 0);
 
    } else if (robot.state == 200) {
      robot.v1_PWM = 0;
      robot.v2_PWM = 0;

    } else if (robot.state == 201) {
      robot.v1_PWM = robot.req1_PWM;
      robot.v2_PWM = robot.req2_PWM;
    
    } else if (robot.state == 202) {
      robot.v1_PWM = robot.req1_PWM;
      robot.v2_PWM = robot.req2_PWM;      
    }

}
