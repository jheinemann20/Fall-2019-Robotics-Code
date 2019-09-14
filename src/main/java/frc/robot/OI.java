/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BuffaloCommands.BuffaloNoseOutCom;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCom;
import frc.robot.commands.ElevatorCommands.ElevatorDownCom;
import frc.robot.commands.ElevatorCommands.ElevatorUpCom;
import frc.robot.commands.HerderCommands.HerderCollect;
import frc.robot.commands.HerderCommands.HerderDispense;
import frc.robot.commands.HerderCommands.TestHerderArmInCom;
import frc.robot.commands.HerderCommands.TestHerderArmOutCom;
import frc.robot.commands.LifterCommands.BothLiftersDownCom;
import frc.robot.commands.LifterCommands.FrontLifterUpCom;
import frc.robot.commands.LifterCommands.LifterDriveReverseCom;
import frc.robot.commands.LifterCommands.RearLifterUpCom;
import frc.robot.commands.LifterCommands.RearPinOutCom;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Declare driver stick and buttons
  private Joystick driveStick;

  private JoystickButton shiftButton;
  private JoystickButton lifterDriveForwardButton;
  private JoystickButton lifterDriveReverseButton;

  // Declare elevator stick and buttons
  private Joystick elevatorStick;

  private JoystickButton elevatorUpButton;
  private JoystickButton elevatorDownButton;

  private JoystickButton buffaloNoseShift;

  private JoystickButton frontPinShift;
  private JoystickButton rearPinShift;

  private JoystickButton collect, dispense;

  private JoystickButton frontLiftUpButton;
  private JoystickButton rearLiftUpButton;
  private JoystickButton bothLiftersDownButton1;
  // Program so both buttons have to be pressed to drop lifter
  private JoystickButton bothLiftersDownButton2;
  private JoystickButton bothLiftersUpButton;

  private JoystickButton herderArmInButton;
  private JoystickButton herderArmOutButton;

  private double xSpeed;
  private double ySpeed;
  private double zRotation;
  private int throttleCurve = 10;

  private DigitalInput leftEye;
  private DigitalInput mideEye;
  private DigitalInput rightEye;

  public OI() {

    // Drive Joystick--------------------------------------------------------
    driveStick = new Joystick(RobotMap.DRIVE_STICK_CH);
    // Lifter code-------------------------------------------------

    frontLiftUpButton = new JoystickButton(driveStick, RobotMap.FRONT_LIFT_UP_BUTTON_CH);
    frontLiftUpButton.whileHeld(new FrontLifterUpCom());

    rearLiftUpButton = new JoystickButton(driveStick, RobotMap.REAR_LIFT_UP_BUTTON_CH);
    rearLiftUpButton.whileHeld(new RearLifterUpCom()); 

    //This code should be updated to require both buttons to be pressed to lower lifters
    bothLiftersDownButton1 = new JoystickButton(driveStick, RobotMap.BOTH_LIFTERS_DOWN_BUTTON_CH_1);
    bothLiftersDownButton1.whileHeld(new BothLiftersDownCom());
    
    shiftButton = new JoystickButton(driveStick, RobotMap.SHIFT_BUTTON_CH);
    shiftButton.toggleWhenPressed(new ArcadeDriveCom());

    

    // lifterDriveReverseButton = new JoystickButton(driveStick, RobotMap.LIFTER_DRIVE_REVERSE_BUTTON_CH);
    // lifterDriveReverseButton.whileHeld(new LifterDriveReverseCom());

  

     

    // End Drive joystick-----------------------------------------------------



    // Elevator Joystick ----------------------------------------------------

    elevatorStick = new Joystick(RobotMap.ELEVATOR_STICK_CH);

    // buffalo nose code---------------------------------------------
    buffaloNoseShift = new JoystickButton(elevatorStick, RobotMap.BUFFALO_NOSE_SHIFT_BTN_CH);
    buffaloNoseShift.toggleWhenPressed(new BuffaloNoseOutCom());

    // frontPinShift = new JoystickButton(elevatorStick, RobotMap.FRONT_LIFTER_PINS_SHIFT_BTTN_CH);
    // frontPinShift.toggleWhenPressed(new FrontPinOutCom());

    rearPinShift = new JoystickButton(elevatorStick, RobotMap.REAR_LIFTER_PINS_SHIFT_BTTN_CH);
    rearPinShift.whileHeld(new RearPinOutCom());

    // Elevator Code
    elevatorUpButton = new JoystickButton(elevatorStick, RobotMap.ELEVATOR_UP_BTN_CH);
    elevatorUpButton.whileHeld(new ElevatorUpCom());

    elevatorDownButton = new JoystickButton(elevatorStick, RobotMap.ELEVATOR_DOWN_BTN_CH);
    elevatorDownButton.whileHeld(new ElevatorDownCom());

    lifterDriveForwardButton = new JoystickButton(elevatorStick, RobotMap.LIFTER_DRIVE_FORWARD_BUTTON_CH);
    lifterDriveForwardButton.whileHeld(new LifterDriveReverseCom());

  

    // Herder code--------------------------------------------------
    collect = new JoystickButton(elevatorStick, RobotMap.HERDER_COLLECT_BTN_CH);
    collect.whileHeld(new HerderCollect());

    dispense = new JoystickButton(elevatorStick, RobotMap.HERDER_DISPENSE_BTN_CH);
    dispense.whileHeld(new HerderDispense());

   

    // Manual Herder Arm Controls (Using Elevator Joystick)
    herderArmInButton = new JoystickButton(elevatorStick, RobotMap.HERDER_ARM_IN_BTN);
    herderArmInButton.whileHeld(new TestHerderArmInCom());

    herderArmOutButton = new JoystickButton(elevatorStick, RobotMap.HERDER_ARM_OUT_BTN);
    herderArmOutButton.whileHeld(new TestHerderArmOutCom());
  }

  // Added option for mapping the drive axis to an exponential curve between 0 and
  // 1
  // by the eq: x*1/y*y^x (where x=joystick and y=scailing factor (throttleCurve))
  // -Andrew
  public double getX() {
    // return ((-driveStick.getRawAxis(1))*(1/throttleCurve)*Math.pow(throttleCurve, (-driveStick.getRawAxis(1))));
    return (-driveStick.getRawAxis(1)) * 0.5;
  }

  public double getY() {
    // return ((driveStick.getRawAxis(0))*(1/throttleCurve)*Math.pow(throttleCurve, (driveStick.getRawAxis(0))));
    return -driveStick.getRawAxis(0) * 0.5;
  }

  public double getTwist() {
    // return ((driveStick.getRawAxis(2))*(1/throttleCurve)*Math.pow(throttleCurve, (driveStick.getRawAxis(2))));
    return (-driveStick.getRawAxis(2)) * 0.5;
  }

  public double getCameraTwist() {
    return elevatorStick.getRawAxis(1);
  }

  public double getCameraYaw() {
    return elevatorStick.getRawAxis(0);
  }

  public double getSecondaryY() {
    return -elevatorStick.getRawAxis(0);
  }

  public double getSecondaryLX() {
    return elevatorStick.getRawAxis(2) * 0.5;
  }

  public double getSecondaryRX() {
    return elevatorStick.getRawAxis(3) * 0.5;
  }

  public boolean getLeftEye() {
    return leftEye.get();
  }

  public boolean getMidEye() {
    return leftEye.get();
  }

  public boolean getRightEye() {
    return leftEye.get();
  }

}
