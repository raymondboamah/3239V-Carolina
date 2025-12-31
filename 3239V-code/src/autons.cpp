#include "vex.h"

// Uses Autonomous Functions from autons.h

// Default Variable Constants ------------------------------------------------------------------------------

const double DISTANCE_BETWEEN_SENSORS = 6.4; // Distance between the two distance sensors in inches.

const double DIST_BACK_OFFSET = 6; // Offset for the back distance sensors in inches, used to correct the position of the robot.
const double DIST_LEFT_OFFSET = 5.37500; // Offset for the left distance sensor in inches, used to correct the position of the robot.
const double DIST_RIGHT_OFFSET = 5.37500; // Offset for the right distance sensor in inches, used to correct the position of the robot.

const double FIELD_LENGTH = 144; // Length of the field in inches.


// Default Drive Constants ---------------------------------------------------------------------------------

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 0.75, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .25, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}


void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

// --------------------------------------------------------------------------------------------------------

// Distance Sensor Functions ------------------------------------------------------------------------------

void heading_checker(double base){
  // Uses 2 distance sensors to check the heading of the robot.
  // It uses the equation: atan((difference_between_sensors)/distance_between_sensors), everything in degrees and inches
  double distBackL = dist_back_left.objectDistance(inches)-0.1;
  double distBackR = dist_back_right.objectDistance(inches);
  double difference = distBackL - distBackR;
  double heading = atan2(difference, DISTANCE_BETWEEN_SENSORS) * (180.0 / M_PI);

  chassis.set_heading(base + heading);
}


// void position_update(){
//   // Updates the position of the robot using the distance sensors and also using the heading so we can be able to accurately
//   // calculate the position of the robot.
//   double heading_angle = chassis.get_absolute_heading();
//   double distBackL = dist_back_left.objectDistance(inches);
//   double distBackR = dist_back_right.objectDistance(inches);
//   double distLeft = dist_left.objectDistance(inches) + DIST_LEFT_OFFSET;
//   double distRight = dist_right.objectDistance(inches) + DIST_RIGHT_OFFSET;
//   double distBack = (distBackL + distBackR) / 2.0 + DIST_BACK_OFFSET;

//   if(heading_angle < 135 && heading_angle > 45){
//     // If the robot is facing to the right, we can use the back sensors to calculate the x position of the bot,
//     // and the left or right sensors to calculate the y position of the bot, depending on which side is closer.
//     if(distLeft < distRight){
//       chassis.set_coordinates(distBack, FIELD_LENGTH-distLeft, chassis.get_absolute_heading());
//     } else {
//       chassis.set_coordinates(distBack, distRight, chassis.get_absolute_heading());
//     }
//   } else if(heading_angle < 225 && heading_angle > 135){
//     // If the robot is facing down, we can use the back sensors to calculate the y position of the bot,
//     // and the left or right sensors to calculate the x position of the bot, depending on which side is closer.
//     if(distLeft < distRight){
//       chassis.set_coordinates(distLeft, FIELD_LENGTH-distBack, chassis.get_absolute_heading());
//     } else {
//       chassis.set_coordinates(FIELD_LENGTH-distRight, FIELD_LENGTH-distBack, chassis.get_absolute_heading());
//     }
//   } else if(heading_angle < 315 && heading_angle > 225){
//     // If the robot is facing to the left, we can use the back sensors to calculate the x position of the bot,
//     // and the left or right sensors to calculate the y position of the bot, depending on which side is closer.
//     if(distLeft < distRight){
//       chassis.set_coordinates(FIELD_LENGTH-distBack, distLeft, chassis.get_absolute_heading());
//     } else {
//       chassis.set_coordinates(FIELD_LENGTH-distBack, FIELD_LENGTH-distRight, chassis.get_absolute_heading());
//     }
//   } else {
//     // If the robot is facing up, we can use the back sensors to calculate the y position of the bot,
//     // and the left or right sensors to calculate the x position of the bot, depending on which side is closer.
//     if(distLeft < distRight){
//       chassis.set_coordinates(distLeft, distBack, chassis.get_absolute_heading());
//     } else {
//       chassis.set_coordinates(FIELD_LENGTH-distRight, distBack, chassis.get_absolute_heading());
//     }
//   }
// }

void distance_drive(double dist){
  // this function checks the distance from the wall to the target distance and corrects the robot for it without using past functions
  double target_distance = dist;
  double current_distance = (dist_back_left.objectDistance(inches) + dist_back_right.objectDistance(inches)) / 2.0;
  while(fabs(current_distance - target_distance) > 0.6){
    current_distance = (dist_back_left.objectDistance(inches) + dist_back_right.objectDistance(inches)) / 2.0;
    double error = target_distance - current_distance;
    chassis.drive_distance(error);
  }
  
}


void distance_pid(double kp, double target_distance){
  // this function checks the distance from the wall to the target distance and corrects the robot for it without using past functions. It drives backwards as the dist sensors are on the back
  double current_distance = (dist_back_left.objectDistance(inches) + dist_back_right.objectDistance(inches)) / 2.0;
  while(fabs(current_distance - target_distance) > 1){
    current_distance = (dist_back_left.objectDistance(inches) + dist_back_right.objectDistance(inches)) / 2.0;
    double error = target_distance - current_distance;
    double drive_voltage = kp * error;
    if(drive_voltage > 12){
      drive_voltage = 12;
    } else if(drive_voltage < -12){
      drive_voltage = -12;
    }
    chassis.drive_with_voltage(drive_voltage, drive_voltage);
  }
  // Stop the motors when target is reached
  chassis.drive_with_voltage(0, 0);
}

// Excecution Functions (Executes the distance sensor functions in match play use)


void angle_check_looper(int base){
  while(chassis.get_absolute_heading() < base - 1 || chassis.get_absolute_heading() > base + 1){
    chassis.turn_to_angle(base);
    heading_checker(base);
    task::sleep(20);
  }
}

void stick_score(double angle){
  // This function uses PID to control the stick motors to a certain position using the rotation sensor to check postion using absolute rotation
  double kP = 0.15;
  double target_angle = angle;
  double current_angle = stick_rot.position(degrees);
  double error = target_angle - current_angle;
  
  // Add timeout to prevent infinite loop (max 2 seconds)
  int timeout_counter = 0;
  while((error > 2.5 || error < -2.5) && timeout_counter < 40){
    current_angle = stick_rot.position(degrees);
    error = target_angle - current_angle;
    double motor_power = kP * error;
    stick.spin(fwd, motor_power, volt);
    wait(20, msec); // Small delay
    timeout_counter++;
  }
  stick.stop();
}


// Odometry Test Function ----------------------------------------------------------------------------------
void odom_test(){
  // position_update();  
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}
// --------------------------------------------------------------------------------------------------------


void sawp(){
  odom_constants();
  chassis.set_coordinates(0, 0, 90);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_settle_error = 5;
  chassis.swing_settle_error = 5;
  hood.set(true);
  distance_pid(0.375, 18);
  matchloading.set(true);

  chassis.turn_to_angle(180);

  chassis.drive_distance(10);
  intake_bot.spin(fwd, 12, volt);
  wait(600, msec);
  chassis.turn_to_angle(180);
  hood.set(false);
  lift.set(true);

  chassis.drive_distance(-28.5);
  chassis.drive_distance(0);
  matchloading.set(false);
  wait(250,msec);


  stick_score(155);

  intake_bot.spin(reverse, 12, volt);

  

  chassis.drive_distance(5);
  stick_score(0);

  
  // hood.set(true);
  chassis.set_drive_constants(10, 0.9, 0, 10, 0);

  chassis.turn_to_angle(65);
  lift.set(false);

  intake_bot.spin(fwd, 12, volt);

  chassis.drive_min_voltage = 8;
  chassis.drive_distance(20);
  matchloading.set(true);
  chassis.drive_distance(10);
  chassis.turn_to_angle(-136);
  chassis.drive_distance(-15);
  chassis.drive_distance(0);
  stick_score(155);
  stick_score(0);
}


void right_double(){
  odom_constants();
  chassis.set_coordinates(0, 0, -90);
  chassis.set_drive_constants(10, 1.25, 0, 10, 0);
  chassis.drive_settle_error = 5;
  distance_pid(0.375, 18);
  matchloading.set(true);
  chassis.turn_to_angle(180);
  chassis.drive_distance(10);
  intake_bot.spin(fwd, 12, volt);
  wait(600, msec);
  chassis.turn_to_angle(180);
  hood.set(false);
  lift.set(true);
  chassis.drive_distance(-28.5);
  chassis.drive_distance(0);
  matchloading.set(false);
  wait(250,msec);
  stick_score(155);
  intake_bot.spin(reverse, 12, volt);
  chassis.drive_distance(5);
  stick_score(0);
  chassis.set_drive_constants(10, 0.9, 0, 10, 0);
  chassis.turn_to_angle(-65);
  intake_bot.spin(fwd, 12, volt);
  chassis.drive_min_voltage = 8;
  chassis.drive_distance(20);
  matchloading.set(true);
  chassis.drive_distance(10);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(15);
  chassis.drive_distance(0);
  intake_bot.spin(reverse, 12, volt);



}


// void skills_auton(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.set_drive_constants(10, 1.25, 0, 10, 0);
//   chassis.drive_settle_error = 5;
  
//   chassis.drive_distance(6);
//   intake_bot.spin(fwd, 12, volt);
//   chassis.turn_to_angle(16.5);

//   chassis.set_drive_constants(10, 0.35, 0, 10, 0);
//   chassis.drive_min_voltage = 5;
//   chassis.drive_distance(17);
//   matchloading.set(true);
//   chassis.drive_distance(3);
//   chassis.drive_min_voltage = 0;
//   chassis.set_drive_constants(10, 0.25, 0, 10, 0);
//   chassis.drive_distance(12);
//   chassis.drive_settle_error = 15;
//   chassis.set_drive_constants(10, 1.35, 0, 10, 0);
//   chassis.drive_distance(0);
//   wait(50, msec);
//   intake_bot.stop();

//   chassis.turn_to_angle(-45);
//   chassis.drive_distance(-20);

//   matchloading.set(false);

//   chassis.turn_to_angle(-90);
//   chassis.drive_distance(-10);
//   distance_drive(19.5);

//   chassis.turn_to_angle(180);
//   chassis.set_drive_constants(10, 1.1, 0, 10, 0);

//   chassis.drive_distance(-16);
//   // chassis.drive_distance(0);

//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);

//   wait(4000, msec);

//   intake_bot.stop();
//   intake_top.stop();

//   chassis.turn_to_angle(180);


//   chassis.set_drive_constants(10, 0.6, 0, 10, 0);


//   matchloading.set(true);

//   chassis.drive_distance(20);

//   chassis.turn_to_angle(-90);
//   distance_drive(21.5);
//   chassis.turn_to_angle(180);

//   chassis.drive_distance(22);

//   // chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   // chassis.drive_distance(4);



//   chassis.turn_to_angle(183);



//   intake_bot.spin(fwd, 12, volt);

//   wait(1100, msec);
//     chassis.drive_distance(4);
//   chassis.drive_distance(0);

//   wait(1100, msec);

//   chassis.turn_to_angle(182.5);

//   chassis.set_drive_constants(10, 0.75, 0, 10, 0);

//   // chassis.drive_distance(-10);
//   //   chassis.drive_distance(0);
//   //   wait(800, msec);
//   //   chassis.turn_to_angle(182.5);
//   //   chassis.drive_distance(13);

//   // chassis.drive_distance(0);
//   //   wait(1000, msec);
    



//   chassis.drive_distance(-22);
//   chassis.turn_to_angle(-90);
//   distance_drive(20);
//   chassis.turn_to_angle(182);
//   chassis.drive_distance(-12);
//   // chassis.drive_distance(0);


//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);

//   wait(4000, msec);

//   matchloading.set(false);
//   intake_bot.stop();
//   intake_top.stop();
//   chassis.drive_distance(10);
//   chassis.turn_to_angle(-150);
//   chassis.drive_distance(35);
//   chassis.turn_to_angle(-100);

//     intake_bot.spin(reverse, 12, volt);

//     chassis.set_drive_constants(10, 1.2, 0, 10, 0);
//   chassis.drive_min_voltage = 8;
//   chassis.drive_distance(7);
//   matchloading.set(true);
//   chassis.drive_distance(40);
//   chassis.drive_distance(0);
//   matchloading.set(false);
//   chassis.drive_distance(10);
//   wait(1000, msec);
//   intake_bot.stop();
//   chassis.drive_min_voltage = 0;


// }


// // Autonomous Action Functions ---------------------------------------------------------------------------


// void six_block_right(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.set_drive_constants(10, 1.25, 0, 10, 0);
//   chassis.drive_settle_error = 5;
  
//   chassis.drive_distance(6);
//   intake_bot.spin(fwd, 12, volt);
//   chassis.turn_to_angle(16.5);

//   chassis.set_drive_constants(10, 0.65, 0, 10, 0);
//   chassis.drive_min_voltage = 5;
//   chassis.drive_distance(17);
//   matchloading.set(true);
//   chassis.drive_distance(3);
//   chassis.drive_min_voltage = 0;
//   chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   chassis.drive_distance(12);
//   chassis.drive_settle_error = 15;
//   chassis.set_drive_constants(10, 1.35, 0, 10, 0);
//   chassis.drive_distance(0);
//   wait(50, msec);
//   intake_bot.stop();

//   chassis.turn_to_angle(-45);
//   chassis.drive_distance(-20);
//   chassis.turn_to_angle(-90);
//   distance_drive(21.5);


//   // matchloading.set(false);

//   // chassis.turn_to_angle(-90);
//   // chassis.drive_distance(-10);
//   // distance_drive(19.5);

//   // chassis.turn_to_angle(180);
//   // chassis.set_drive_constants(10, 1.1, 0, 10, 0);

//   // chassis.drive_distance(-16);
//   // // chassis.drive_distance(0);

//   // intake_bot.spin(fwd, 12, volt);
//   // intake_top.spin(fwd, 12, volt);

//   // wait(1400, msec);

//   // intake_bot.stop();
//   // intake_top.stop();

//   chassis.turn_to_angle(180);


//   chassis.set_drive_constants(10, 0.45, 0, 10, 0);


//   matchloading.set(true);

//   // chassis.drive_distance(20);

//   // chassis.turn_to_angle(-90);
//   chassis.turn_to_angle(180);

//   chassis.drive_distance(32);

//   // chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   // chassis.drive_distance(4);



//   chassis.turn_to_angle(183);

//   intake_bot.spin(fwd, 12, volt);

//   wait(1100, msec);


//   chassis.turn_to_angle(185.5);

//   chassis.set_drive_constants(10, 0.75, 0, 10, 0);

//   // chassis.drive_distance(-10);
//   //   chassis.drive_distance(0);
//   //   wait(800, msec);
//   //   chassis.turn_to_angle(182.5);
//   //   chassis.drive_distance(13);

//   // chassis.drive_distance(0);
//   //   wait(1000, msec);
    



//   // chassis.drive_distance(-20);
//   // chassis.turn_to_angle(-90);
//   // distance_drive(20);
//   // chassis.turn_to_angle(182);
//   // chassis.drive_distance(-12);

//   chassis.drive_distance(-32);
//   // chassis.drive_distance(0);


//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);

// }

// void two_goal_left(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.set_drive_constants(10, 1.25, 0, 10, 0);
//   chassis.drive_settle_error = 5;
  
//   chassis.drive_distance(6);
//   intake_bot.spin(fwd, 12, volt);
//   chassis.turn_to_angle(-16.5);

//   chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   chassis.drive_min_voltage = 5;
//   chassis.drive_distance(17);
//   matchloading.set(true);
//   chassis.drive_distance(4.5);
//   chassis.drive_min_voltage = 0;
//   chassis.set_drive_constants(10, 0.45, 0, 10, 0);
//   chassis.drive_distance(8);
//   chassis.drive_settle_error = 15;
//   chassis.set_drive_constants(10, 1.35, 0, 10, 0);
//   chassis.drive_distance(0);
//   wait(50, msec);
//   intake_bot.stop();

//   chassis.turn_to_angle(-135);
//   middle_goal_loading.set(true);

//   chassis.drive_distance(-20);
//   intake_bot.spin(reverse, 12, volt);
//   wait(250, msec);
//   intake_bot.spin(fwd, 12, volt);
//   wait(1350, msec);
//   middle_goal_loading.set(false);
//   intake_bot.stop();
//   chassis.drive_distance(35);
//   chassis.turn_to_angle(90);
//   // chassis.turn_to_angle(90);
//   distance_drive(22);


//   // matchloading.set(false);

//   // chassis.turn_to_angle(-90);
//   // chassis.drive_distance(-10);
//   // distance_drive(19.5);

//   // chassis.turn_to_angle(180);
//   // chassis.set_drive_constants(10, 1.1, 0, 10, 0);

//   // chassis.drive_distance(-16);
//   // // chassis.drive_distance(0);

//   // intake_bot.spin(fwd, 12, volt);
//   // intake_top.spin(fwd, 12, volt);

//   // wait(1400, msec);

//   // intake_bot.stop();
//   // intake_top.stop();

//   chassis.turn_to_angle(180);


//   chassis.set_drive_constants(10, 0.27, 0, 10, 0);


//   matchloading.set(true);

//   // chassis.drive_distance(20);

//   // chassis.turn_to_angle(-90);
//   chassis.turn_to_angle(182);

//   chassis.drive_distance(33);

//   // chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   // chassis.drive_distance(4);



//   // chassis.turn_to_angle(-183);



//   intake_bot.spin(fwd, 12, volt);

//   wait(650, msec);


//   chassis.turn_to_angle(-180);

//   chassis.set_drive_constants(10, 0.75, 0, 10, 0);

//   // chassis.drive_distance(-10);
//   //   chassis.drive_distance(0);
//   //   wait(800, msec);
//   //   chassis.turn_to_angle(182.5);
//   //   chassis.drive_distance(13);

//   // chassis.drive_distance(0);
//   //   wait(1000, msec);
    



//   // chassis.drive_distance(-20);
//   // chassis.turn_to_angle(-90);
//   // distance_drive(20);
//   // chassis.turn_to_angle(182);
//   // chassis.drive_distance(-12);

//   chassis.drive_distance(-32);
//   // chassis.drive_distance(0);


//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);

// }

// void six_block_left(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.set_drive_constants(10, 1.25, 0, 10, 0);
//   chassis.drive_settle_error = 5;
  
//   chassis.drive_distance(6);
//   intake_bot.spin(fwd, 12, volt);
//   chassis.turn_to_angle(-16.5);

//   chassis.set_drive_constants(10, 0.7, 0, 10, 0);
//   chassis.drive_min_voltage = 5;
//   chassis.drive_distance(17);
//   matchloading.set(true);
//   chassis.drive_distance(3);
//   chassis.drive_min_voltage = 0;
//   chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   chassis.drive_distance(6);
//   chassis.drive_settle_error = 15;
//   chassis.set_drive_constants(10, 1.35, 0, 10, 0);
//   chassis.drive_distance(0);
//   wait(50, msec);
//   intake_bot.stop();

//   chassis.turn_to_angle(45);
//   chassis.drive_distance(-18);
//   chassis.turn_to_angle(90);
//   distance_drive(21.5);


//   // matchloading.set(false);

//   // chassis.turn_to_angle(-90);
//   // chassis.drive_distance(-10);
//   // distance_drive(19.5);

//   // chassis.turn_to_angle(180);
//   // chassis.set_drive_constants(10, 1.1, 0, 10, 0);

//   // chassis.drive_distance(-16);
//   // // chassis.drive_distance(0);

//   // intake_bot.spin(fwd, 12, volt);
//   // intake_top.spin(fwd, 12, volt);

//   // wait(1400, msec);

//   // intake_bot.stop();
//   // intake_top.stop();

//   chassis.turn_to_angle(180);


//   chassis.set_drive_constants(10, 0.35, 0, 10, 0);

//     intake_bot.spin(fwd, 12, volt);

//   matchloading.set(true);

//   // chassis.drive_distance(20);

//   // chassis.turn_to_angle(-90);
//   chassis.turn_to_angle(180);

//   chassis.drive_distance(32);

//   // chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   // chassis.drive_distance(4);



//   chassis.turn_to_angle(-183);

//   wait(450, msec);

//   chassis.turn_to_angle(-181.5);

//   chassis.set_drive_constants(10, 0.75, 0, 10, 0);

//   // chassis.drive_distance(-10);
//   //   chassis.drive_distance(0);
//   //   wait(800, msec);
//   //   chassis.turn_to_angle(182.5);
//   //   chassis.drive_distance(13);

//   // chassis.drive_distance(0);
//   //   wait(1000, msec);
    



//   // chassis.drive_distance(-20);
//   // chassis.turn_to_angle(-90);
//   // distance_drive(20);
//   // chassis.turn_to_angle(182);
//   // chassis.drive_distance(-12);

//   chassis.drive_distance(-32);
//   // chassis.drive_distance(0);


//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);
// }


// void right_control(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.set_drive_constants(10, 1.25, 0, 10, 0);
//   chassis.drive_settle_error = 5;
  
//   chassis.drive_distance(6);
//   intake_bot.spin(fwd, 12, volt);
//   chassis.turn_to_angle(16.5);

//   chassis.set_drive_constants(10, 0.65, 0, 10, 0);
//   chassis.drive_min_voltage = 5;
//   chassis.drive_distance(17);
//   matchloading.set(true);
//   chassis.drive_distance(3);
//   chassis.drive_min_voltage = 0;
//   chassis.set_drive_constants(10, 0.5, 0, 10, 0);
//   chassis.drive_distance(12);
//   chassis.drive_settle_error = 15;
//   chassis.set_drive_constants(10, 1.35, 0, 10, 0);
//   chassis.drive_distance(0);
//   wait(50, msec);
//   // intake_bot.stop();

//   chassis.turn_to_angle(-45);
//   chassis.drive_distance(-20);
//   chassis.turn_to_angle(-90);
//   distance_drive(20.5);
//   chassis.turn_to_angle(181.5);

//   chassis.drive_distance(-10);
//   chassis.drive_distance(0);
//   intake_bot.spin(fwd, 12, volt);
//   intake_top.spin(fwd, 12, volt);
//   wait(2000, msec);

//   intake_bot.stop();
//   intake_top.stop();

//   chassis.drive_distance(5);
//   chassis.turn_to_angle(-90);

//   distance_drive(30);
//   chassis.turn_to_angle(180);

//   chassis.drive_distance(-33);
//   chassis.drive_distance(0);


// }