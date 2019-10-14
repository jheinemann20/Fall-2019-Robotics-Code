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
import frc.robot.commands.HerderCommands.HerderStopCollect;

/**
 * Add your docs here.
 */
public class HerderSub extends Subsystem {
 
  private WPI_VictorSPX herderCollectorTop;
  private WPI_VictorSPX herderCollectorBottom;

  public HerderSub() {

    // create herder motor (VictorSPX)

    herderCollectorTop = new WPI_VictorSPX(RobotMap.HERDER_TOP_COLLECTOR_CHANNEL);
    herderCollectorTop.configOpenloopRamp(0);

    herderCollectorBottom = new WPI_VictorSPX(RobotMap.HERDER_BOTTOM_COLLECTOR_CHANNEL);
    herderCollectorBottom.configOpenloopRamp(0);

    

  }

  public void collect() {
    herderCollectorBottom.set(1);
    herderCollectorTop.set(1);
  }

  public void dispense() {
    herderCollectorBottom.set(-1);
    herderCollectorTop.set(-1);
  }

  public void herderCollectStop() {
    herderCollectorBottom.set(0);
    herderCollectorTop.set(0);
  }

 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HerderStopCollect());
  }
   
}
