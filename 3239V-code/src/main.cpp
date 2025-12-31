#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,
//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(L1, L2, L3),

//Right Motors:
motor_group(R1, R2, R3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT11,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT1,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
-1.925,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0.25,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT10,

//Sideways tracker diameter (reverse to make the direction switch):
-1.925,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
4.5

);

int current_auton_selection = 0;
bool auto_started = false;

// Toggle variables
bool toggle_matchloading = false;
bool toggle_lift = false;
bool toggle_hood = false;
bool toggle_descore = false;
bool toggle_stick_up = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
        stick_rot.setPosition(0, degrees); // Reset the stick rotation sensor to 0 degrees at the start of user control.
      stick_rot.setReversed(true);

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Prog Skills");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "6 Block Right");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "6 Block Left");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Two Goal Left");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0: 
    // skills_auton();
    // two_goal_left();
    // six_block_left();
    // six_block_right();
        // right_control();
        // sawp();
        right_double();

      break;
    case 1:  
    // six_block_right();
      break;
    case 2:
    // six_block_left();
      break;
    case 3:
    // two_goal_left();
      break;
    case 4:
    // right_control();
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


// User Control Functions


void intake_control() {
    // This function controls the intake motors based on button input.
    // R1 loads balls into the middle goal, R2 unloads balls from the middle goal,
    // L1 loads balls into the top goal
    // L2 only runs the bottom intake motor, used to load balls into the robot

    if(Controller1.ButtonR1.pressing()){
      intake_bot.spin(fwd, 12, volt);
    } else if(Controller1.ButtonR2.pressing()){
      intake_bot.spin(reverse, 12, volt);
    }else{
      intake_bot.stop();
    }
}

void stick_control(){
  // This function uses PID to control the stick motors to a certain position using the rotation sensor to check postion using absolute rotation
  
  double kP = 0.12;
  double target_angle = 0.0;
  if(Controller1.ButtonL1.pressing()){
    target_angle = 155; // Target angle for up position
    hood.set(false);
  }

  double current_angle = stick_rot.position(degrees);
  double error = target_angle - current_angle;
  double motor_power = kP * error;

  // Check if we're close enough to target (within 1 degree)
  if(error > 1 || error < -1){
    stick.spin(fwd, motor_power, volt);
  } else {
    stick.stop();
  }



}

void matchload_control() {
  // This function is a button toggle for the matchloading pneumatic on button A.
  if(Controller1.ButtonA.pressing()) {
    if(!toggle_matchloading) {
      toggle_matchloading = true;
      matchloading.set(true);
      // Add a delay to allow the pneumatic to fully extend
      task::sleep(200); // Adjust the delay as needed
    } else {
      toggle_matchloading = false;
      matchloading.set(false);
      task::sleep(200); // Adjust the delay as needed
    }
  }
}



void descore() {
  // This function is a button toggle for the matchloading pneumatic on button A.
  if(Controller1.ButtonL2.pressing()) {
    if(!toggle_descore) {
      toggle_descore = true;
      descore_pneumatic.set(true);
      // Add a delay to allow the pneumatic to fully extend
      task::sleep(200); // Adjust the delay as needed
    } else {
      toggle_descore = false;
      descore_pneumatic.set(false);
      task::sleep(200); // Adjust the delay as needed
    }
  }
}


void lift_control() {
  if(Controller1.ButtonB.pressing()) {
    if(!toggle_lift) {
      toggle_lift = true;
      lift.set(true);
      // Add a delay to allow the pneumatic to fully extend
      task::sleep(200); // Adjust the delay as needed
    } else {
      toggle_lift = false;
      lift.set(false);
      task::sleep(200); // Adjust the delay as needed
    }
  }
}





void hood_control() {
  // This function is a button toggle for the middle goal loading pneumatic on button X.
  if(Controller1.ButtonX.pressing()) {
    if(!toggle_hood) {
      toggle_hood = true;
      hood.set(true);
      task::sleep(200); // Adjust the delay as needed
    } else {
      toggle_hood = false;
      hood.set(false);
      task::sleep(200); // Adjust the delay as needed

    }
  }
}














// User control function. This function will run during the user control phase of a VEX Competition.
void usercontrol(void) {
  // User control code here, inside the loop
      descore_pneumatic.set(true);


  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // odom_test();   

    stick_control();
    intake_control(); // Call the intake control function to handle intake motors.
    matchload_control(); // Call the match loading control function to handle pneumatics.
    lift_control();
    hood_control();
    descore();


    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}