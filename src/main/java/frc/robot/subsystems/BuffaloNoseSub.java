/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.BuffaloCommands.BuffaloNoseInCom;

/**
 * Add your docs here.
 */
public class BuffaloNoseSub extends Subsystem {

  private DoubleSolenoid buffaloNose;

  public BuffaloNoseSub() {

    buffaloNose = new DoubleSolenoid(RobotMap.BUFFALO_NOSE_FWD, RobotMap.BUFFALO_NOSE_BWD);
  }

  public void buffaloNoseIn() {
    buffaloNose.set(DoubleSolenoid.Value.kForward);
  }

  public void buffaloNoseOut() {
    buffaloNose.set(DoubleSolenoid.Value.kReverse);
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new BuffaloNoseInCom());
  }
}
