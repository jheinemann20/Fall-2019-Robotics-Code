/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.LifterCommands.LifterDriveStopCom;


/**
 * Add your docs here.
 */
public class LifterDriveSub extends Subsystem {
  // Initialize all of the objects and values for the
  // subsystem-----------------------------------------------------------------------

  // lift drive motor
  private WPI_VictorSPX liftDriveMotor;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public LifterDriveSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem

    // Lift driver motor
    liftDriveMotor = new WPI_VictorSPX(RobotMap.LIFT_DRIVE_MOTOR_CH);

  }

  // Lifter drive motor methods---------------------------------------------
  public void lifterDriveForward() {
    liftDriveMotor.set(-1);
  }

  public void lifterDriveReverse() {

    liftDriveMotor.set(1);
  }

  public void lifterDriveStop() {
    liftDriveMotor.set(0);
  }

  // PID setpoint methods ---------------------------------------------------



  // This method sets the default command to stop motors- this causes motors to
  // default to stop during normal robot operation
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LifterDriveStopCom());
  }
}