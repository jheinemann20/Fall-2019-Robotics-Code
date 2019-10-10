/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LifterCommands.StopLifterMotorsCom;

/**
 * Add your docs here.
 */
public class LifterSub extends Subsystem {
  // Initialize all of the objects and values for the
  // subsystem-----------------------------------------------------------------------

  // front lifter
  private WPI_VictorSPX frontLifterMotor;
  private AnalogInput frontLiftPot;
  private DigitalInput frontTopLiftLimit;
  private DigitalInput frontBottomLiftLimit;
  private PIDController frontLiftPID;

  // rear lifter
  private WPI_VictorSPX rearLifterMotor;
  private AnalogInput rearLiftPot;
  private DigitalInput rearTopLiftLimit;
  private DigitalInput rearBottomLiftLimit;
  private PIDController rearLiftPID;

  // pid coefficients
  private double kP, kI, kD, kMaxOutput, kMinOutput;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public LifterSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem

    // Front lifter
    frontLifterMotor = new WPI_VictorSPX(RobotMap.FRONT_LIFT_MOTOR_CH);
    frontLiftPot = new AnalogInput(RobotMap.FRONT_LIFT_POT_CH);
    frontTopLiftLimit = new DigitalInput(RobotMap.FRONT_TOP_LIFT_LIMIT_CH);
    frontBottomLiftLimit = new DigitalInput(RobotMap.FRONT_BOTTOM_LIFT_LIMIT_CH);

    // Rear lifter
    rearLifterMotor = new WPI_VictorSPX(RobotMap.REAR_LIFT_MOTOR_CH);
    rearLiftPot = new AnalogInput(RobotMap.REAR_LIFT_POT_CH);
    rearTopLiftLimit = new DigitalInput(RobotMap.REAR_TOP_LIFT_LIMIT_CH);
    rearBottomLiftLimit = new DigitalInput(RobotMap.REAR_BOTTOM_LIFT_LIMIT_CH);

    // create pidcontroller
    frontLiftPID = new PIDController(0, 0, 0, frontLiftPot, frontLifterMotor);
    rearLiftPID = new PIDController(0, 0, 0, rearLiftPot, rearLifterMotor);

    // create PID coefficients
    kP = 0.3;
    kI = 0;
    kD = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // set PID coefficients
    // Front Lifter PID
    frontLiftPID.setP(kP);
    frontLiftPID.setI(kI);
    frontLiftPID.setD(kD);
    frontLiftPID.setOutputRange(kMinOutput, kMaxOutput);
    // Rear Lifter PID
    rearLiftPID.setP(kP);
    rearLiftPID.setI(kI);
    rearLiftPID.setD(kD);
    rearLiftPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  // end of constructor--------------------------------------------------------

  // methods for printing values to SmartDashboard-----------------------------

  public void getAllLiftSensors() {
    getFrontLiftPot();
    getRearLiftPot();
    getFrontLiftTopLimit();
    getFrontLiftTopLimit();
    getRearLiftTopLimit();
    getRearLiftBottomLimit();
  }

  public double getFrontLiftPot() {
    SmartDashboard.putNumber("Front Lift Pot Voltage", frontLiftPot.getVoltage());
    return frontLiftPot.getVoltage();
  }

  public double getRearLiftPot() {
    SmartDashboard.putNumber("Rear Lift Pot Voltage", rearLiftPot.getVoltage());
    return rearLiftPot.getVoltage();
  }

  public boolean getFrontLiftTopLimit() {
    SmartDashboard.putBoolean("Front Top Lift Limit", frontTopLiftLimit.get());
    return frontTopLiftLimit.get();
  }

  public boolean getFrontLiftBottomLimit() {
    SmartDashboard.putBoolean("Front Bottom Lift Limit", frontBottomLiftLimit.get());
    return frontBottomLiftLimit.get();
  }

  public boolean getRearLiftTopLimit() {
    SmartDashboard.putBoolean("Rear Top Lift Limit", rearTopLiftLimit.get());
    return rearTopLiftLimit.get();
  }

  public boolean getRearLiftBottomLimit() {
    SmartDashboard.putBoolean("Rear Bottom Lift Limit", rearBottomLiftLimit.get());
    return rearBottomLiftLimit.get();
  }

  // Methods for moving lifters up
  // ------------------------------------------------------------------

  // Front lifter up
  public void frontLifterUp() {
    // double frontLiftPotValue = getFrontLiftPot();
    // boolean frontTopLimitValue = getFrontLiftTopLimit();

    // if (frontLiftPotValue < 2.88) {
    //   frontLifterMotor.set(-.5);
    // }

    // else {
    //   frontLifterMotor.set(0);
    // }
    frontLifterMotor.set(-.5);
  }

  // Rear lifter up
  public void rearLifterUp() {
    // Robot.lifterPinSub.rearPinsOut();
    // double rearLiftPotValue = getRearLiftPot();
    // boolean rearTopLimitValue = getRearLiftTopLimit();

    // if (rearLiftPotValue < 2.98) {
    //   rearLifterMotor.set(.5);
    // }

    // else {
    //   rearLifterMotor.set(0);
    // }
    rearLifterMotor.set(.5);
  }

  // Both lifters down--------------------------------------------------

  public void bothLiftersDown() {
    // double frontLiftPotValue = getFrontLiftPot();
    // double rearLiftPotValue = getRearLiftPot();
    // boolean frontBottomLimitValue = getFrontLiftTopLimit();
    // boolean rearBottomLimitValue = getRearLiftTopLimit();

    // When front bottom limit is online add to this statement
    // if (frontLiftPotValue > 0.8 && rearLiftPotValue > 0.9) {

    //   if (frontLiftPotValue < rearLiftPotValue) {
    //     frontLifterMotor.set(.5);
    //     rearLifterMotor.set(.7);
    //   }

    //   else {
    //     frontLifterMotor.set(.7);
    //     rearLifterMotor.set(.5);
    //   }

    // }

    // if (frontLiftPotValue > 0.2) {
    //   frontLifterMotor.set(.5);
    // } else {
    //   frontLifterMotor.set(0);
    //   Robot.lifterDriveSub.lifterDriveForward(); // Added drive forward after front lifter gets to the top. -Andrew
    // }

    // if (frontLiftPotValue > 0.2) {
    //   if (frontLiftPotValue < rearLiftPotValue) {
    //     frontLifterMotor.set(.3);
    //   } else {
    //     frontLifterMotor.set(.6);
    //   }
    // } else {
    //   frontLifterMotor.set(0);
    //   Robot.lifterDriveSub.lifterDriveForward(); // Added drive forward after front lifter gets to the top. -Andrew
    // }

    // if (rearLiftPotValue > 0.2) {
    //   Robot.lifterPinSub.rearPinsOut();
    //   if (frontLiftPotValue < rearLiftPotValue) {
    //     rearLifterMotor.set(-0.6);
    //   } else {
    //     rearLifterMotor.set(-0.4);
    //   }
    // } else {
    //   rearLifterMotor.set(0);
    //   Robot.lifterPinSub.rearPinsIn();
    // }

    frontLifterMotor.set(.5);
    rearLifterMotor.set(-.5);
  }

  // Method to call for default commmand to keep motors still during teleop
  public void stopLiftMotors() {
    // Robot.lifterPinSub.rearPinsIn();
    frontLifterMotor.set(0);
    rearLifterMotor.set(0);
  }

  // PID setpoint methods ---------------------------------------------------

  public void setFrontLifterPID(double setPoint) {
    frontLiftPID.setSetpoint(setPoint);
  }

  public void setRearLifterPID(double setPoint) {
    rearLiftPID.setSetpoint(setPoint);
  }

  // This method sets the default command to stop motors- this causes motors to
  // default to stop during normal robot operation
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new StopLifterMotorsCom());
  }
}