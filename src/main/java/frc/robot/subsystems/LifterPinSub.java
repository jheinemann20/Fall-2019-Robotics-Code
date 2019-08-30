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
import frc.robot.commands.LifterCommands.RearPinInCom;
import frc.robot.commands.LifterCommands.RearPinOutCom;

/**
 * Add your docs here.
 */
public class LifterPinSub extends Subsystem {
  // private DoubleSolenoid frontPins;
  private DoubleSolenoid rearPins;

  public LifterPinSub() {
    // frontPins = new DoubleSolenoid(RobotMap.FRONT_LIFTER_PIN_IN_CH, RobotMap.FRONT_LIFTER_PIN_OUT_CH);
    rearPins = new DoubleSolenoid(RobotMap.REAR_LIFTER_PIN_IN_CH, RobotMap.REAR_LIFTER_PIN_OUT_CH);
  }

  public void frontPinsIn() {
    // frontPins.set(DoubleSolenoid.Value.kForward);
  }

  public void frontPinsOut() {
    // frontPins.set(DoubleSolenoid.Value.kReverse);
  }

  public void rearPinsIn() {
    rearPins.set(DoubleSolenoid.Value.kReverse);
  }

  public void rearPinsOut() {
    rearPins.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new RearPinOutCom());
  }
}
