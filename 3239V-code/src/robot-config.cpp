#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

// Declare the motors from robot-config.h
controller Controller1 = controller();


// Drivetrain Motors
motor L1 = motor(PORT7, ratio6_1, true);
motor L2 = motor(PORT6, ratio6_1, true);
motor L3 = motor(PORT5, ratio6_1, true);

motor R1 = motor(PORT4, ratio6_1, false);
motor R2 = motor(PORT3, ratio6_1, false);
motor R3 = motor(PORT2, ratio6_1, false);

// Intake Motors
motor stick = motor(PORT1, ratio36_1, true);
motor intake_bot1 = motor(PORT8, ratio18_1, false);
motor intake_bot2 = motor(PORT10, ratio18_1, true);

rotation stick_rot = rotation(PORT21, false);

motor_group intake_bot = motor_group(intake_bot1, intake_bot2);



// Distance Sensors
distance dist_back_left = distance(PORT20);
distance dist_back_right = distance(PORT12);


// Pneumatics
digital_out matchloading = digital_out(Brain.ThreeWirePort.G);
digital_out hood = digital_out(Brain.ThreeWirePort.H);
digital_out lift = digital_out(Brain.ThreeWirePort.F);
digital_out descore_pneumatic = digital_out(Brain.ThreeWirePort.C);




void vexcodeInit( void ) {
  // nothing to initialize
}