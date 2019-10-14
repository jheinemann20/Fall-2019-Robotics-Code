/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.OI;

public class ElevatorUpCom extends Command {
  public ElevatorUpCom() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevatorSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevatorSub.elevatorUp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevatorSub.elevatorUp();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.elevatorSub.getTopLimit();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevatorSub.elevatorStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
