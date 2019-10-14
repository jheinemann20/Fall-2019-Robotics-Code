/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
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
  private PIDController frontLiftPID;

  // rear lifter
  private WPI_VictorSPX rearLifterMotor;
  private PIDController rearLiftPID;

  // rear limits
  public DigitalInput rearDownLimit;

  // pid coefficients
  // private double kP, kI, kD, kMaxOutput, kMinOutput;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public LifterSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem

    // Front lifter
    frontLifterMotor = new WPI_VictorSPX(RobotMap.FRONT_LIFT_MOTOR_CH);

    // Rear lifter
    rearLifterMotor = new WPI_VictorSPX(RobotMap.REAR_LIFT_MOTOR_CH);

    // rear limits
    rearDownLimit = new DigitalInput(RobotMap.REAR_BOTTOM_LIFT_LIMIT_CH);

    // // create pidcontroller
    // frontLiftPID = new PIDController(0, 0, 0, frontLiftPot, frontLifterMotor);
    // rearLiftPID = new PIDController(0, 0, 0, rearLiftPot, rearLifterMotor);

    // // create PID coefficients
    // kP = 0.3;
    // kI = 0;
    // kD = 0;
    // kMaxOutput = 0.5;
    // kMinOutput = -0.5;

    // // set PID coefficients
    // // Front Lifter PID
    // frontLiftPID.setP(kP);
    // frontLiftPID.setI(kI);
    // frontLiftPID.setD(kD);
    // frontLiftPID.setOutputRange(kMinOutput, kMaxOutput);
    // // Rear Lifter PID
    // rearLiftPID.setP(kP);
    // rearLiftPID.setI(kI);
    // rearLiftPID.setD(kD);
    // rearLiftPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  // end of constructor--------------------------------------------------------

  // methods for printing values to SmartDashboard-----------------------------

  // public void getAllLiftSensors() {
  //   getFrontLiftPot();
  //   getRearLiftPot();
  //   getFrontLiftTopLimit();
  //   getFrontLiftTopLimit();
  //   getRearLiftTopLimit();
  //   getRearLiftBottomLimit();
  // }

  // public double getFrontLiftPot() {
  //   SmartDashboard.putNumber("Front Lift Pot Voltage", frontLiftPot.getVoltage());
  //   return frontLiftPot.getVoltage();
  // }

  // public double getRearLiftPot() {
  //   SmartDashboard.putNumber("Rear Lift Pot Voltage", rearLiftPot.getVoltage());
  //   return rearLiftPot.getVoltage();
  // }

  // public boolean getFrontLiftTopLimit() {
  //   SmartDashboard.putBoolean("Front Top Lift Limit", frontTopLiftLimit.get());
  //   return frontTopLiftLimit.get();
  // }

  // public boolean getFrontLiftBottomLimit() {
  //   SmartDashboard.putBoolean("Front Bottom Lift Limit", frontBottomLiftLimit.get());
  //   return frontBottomLiftLimit.get();
  // }

  // public boolean getRearLiftTopLimit() {
  //   SmartDashboard.putBoolean("Rear Top Lift Limit", rearTopLiftLimit.get());
  //   return rearTopLiftLimit.get();
  // }

  // public boolean getRearLiftBottomLimit() {
  //   SmartDashboard.putBoolean("Rear Bottom Lift Limit", rearBottomLiftLimit.get());
  //   return rearBottomLiftLimit.get();
  // }

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
    frontLifterMotor.set(-1);
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
    Robot.lifterPinSub.rearPinsOut();
    rearLifterMotor.set(.5);
  }

    // Front lifter down
    public void frontLifterDown() {
      frontLifterMotor.set(1);
    }
  
    // Rear lifter down
    public void rearLifterDown() {
      Robot.lifterPinSub.rearPinsOut();
    if (rearDownLimit.get())
      rearLifterMotor.set(-.5);
    else
      rearLifterMotor.set(0);
    }

  // Both lifters down--------------------------------------------------

  public void bothLiftersDown(double secondSpeed) {
    Robot.lifterPinSub.rearPinsOut();
    frontLifterMotor.set(1);
    if ((secondSpeed > 0) && (rearDownLimit.get()))
      rearLifterMotor.set(-secondSpeed);
    else
      rearLifterMotor.set(0);
  }

  // Method to call for default commmand to keep motors still during teleop
  public void stopLiftMotors() {
    // Robot.lifterPinSub.rearPinsIn();
    frontLifterMotor.set(0);
    rearLifterMotor.set(0);
    Robot.lifterPinSub.rearPinsIn();
  }

  public void stopBackMotor() {
    rearLifterMotor.set(0);
    Robot.lifterPinSub.rearPinsIn();
  }

  public void stopFrontMotor() {
    frontLifterMotor.set(0);
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