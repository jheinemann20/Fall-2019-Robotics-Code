/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // Create constants for the drive motors- change the values to the motor
  // controller ID
  public static int FRONT_LEFT_CHANNEL = 2;
  public static int FRONT_RIGHT_CHANNEL = 4;
  public static int REAR_RIGHT_CHANNEL = 3;
  public static int REAR_LEFT_CHANNEL = 12;

  // Create constants for the elevator motor and limits-------------------
  public static int ELEVATOR_MOTOR_CHANNEL = 9;
  public static int UPPER_ELEVATOR_LIMIT_CHANNEL = 1;
  public static int LOWER_ELEVATOR_LIMIT_CHANNEL = 3;

  // Herder constants------------------------------------------------------
  // Create constants for herder motors
  public static int HERDER_BOTTOM_COLLECTOR_CHANNEL = 7;
  public static int HERDER_TOP_COLLECTOR_CHANNEL = 8;

  // Create constants for the herder arm
  public static int HERDER_ARM_MOTOR_CHANNEL = 10;
  public static int HERDER_POT_CH = 3;
    
  // Buffalo nose----------------------------------------------------------

  // Create constants for the buffalo nose
  public static int BUFFALO_NOSE_FWD = 2;
  public static int BUFFALO_NOSE_BWD = 3;

  // Create constants for servo PWM location
  public static int TRAVERSE_SERVO_CHANNEL = 4;
  public static int YAW_SERVO_CHANNEL = 5;

  // Lifter --------------------------------------------------------------
  // Create the constants for the front lifter
  public static int FRONT_LIFT_MOTOR_CH = 11;
  public static int FRONT_TOP_LIFT_LIMIT_CH = 0;
  public static int FRONT_BOTTOM_LIFT_LIMIT_CH = 2;
  public static int FRONT_LIFT_POT_CH = 0;

  // Create the constants for the rear lifter
  public static int REAR_LIFT_MOTOR_CH = 6;
  public static int REAR_TOP_LIFT_LIMIT_CH = 8;
  public static int REAR_BOTTOM_LIFT_LIMIT_CH = 9;
  public static int REAR_LIFT_POT_CH = 1;

  // Create the constants for the lift drive motor
  public static int LIFT_DRIVE_MOTOR_CH = 5;


  // Create constants for the drive solenoid channels- these should stay the same
  public static int DRIVE_SOL_FORWARD_CH = 0;
  public static int DRIVE_SOL_REVERSE_CH = 1;


  // Create the constants for the driver joystick and buttons--------------

  public static int DRIVE_STICK_CH = 0;
  public static int SHIFT_BUTTON_CH = 1;

  public static int BUFFALO_NOSE_SHIFT_BTN_CH = 7;

  public static int LIFTER_DRIVE_FORWARD_BUTTON_CH = 6;
  public static int LIFTER_DRIVE_REVERSE_BUTTON_CH = 8;


  // Create the constants for the elevator joystick and buttons -----------

  public static int ELEVATOR_STICK_CH = 1;

  // herder arm system
  public static int HERDER_ARM_IN_BTN = 5;
  public static int HERDER_ARM_OUT_BTN = 3;

  // elevator arm system
  public static int ELEVATOR_UP_BTN_CH = 9;
  public static int ELEVATOR_DOWN_BTN_CH = 10;

  // herder system
  public static int HERDER_COLLECT_BTN_CH = 1;
  public static int HERDER_DISPENSE_BTN_CH = 2;

  // lifter system
  public static int BOTH_LIFTERS_DOWN_BUTTON_CH_1 = 6;
  public static int BOTH_LIFTERS_DOWN_BUTTON_CH_2 = 10;

  public static int REAR_LIFT_UP_BUTTON_CH = 10;
  public static int FRONT_LIFT_UP_BUTTON_CH = 9;

  public static int FRONT_LIFTER_PIN_IN_CH = 4;
  public static int FRONT_LIFTER_PIN_OUT_CH = 5;

  public static int REAR_LIFTER_PIN_IN_CH = 4;
  public static int REAR_LIFTER_PIN_OUT_CH = 5;

  public static int FRONT_LIFTER_PINS_SHIFT_BTTN_CH = 6;
  
  public static int REAR_LIFTER_PINS_SHIFT_BTTN_CH = 4; // (currently on DriveStick)


  // Create constant for deadband
  public static double DEADBAND_CH = 0.09;
}
