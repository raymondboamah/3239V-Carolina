using namespace vex;

extern brain Brain;

extern controller Controller1;

// Drivetrain Motors
extern motor L1;
extern motor L2;
extern motor L3;

extern motor R1;
extern motor R2;
extern motor R3;

//Intake Motors
extern motor stick;
extern rotation stick_rot;

extern motor intake_bot1;
extern motor intake_bot2;

extern motor_group intake_bot;



// Distance Sensors

extern distance dist_back_left;
extern distance dist_back_right;
extern distance dist_left;
extern distance dist_right;

// Pneumatics
extern digital_out matchloading;
extern digital_out lift;
extern digital_out hood;
extern digital_out descore_pneumatic;




void  vexcodeInit( void );