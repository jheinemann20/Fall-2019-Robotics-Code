/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommands.ElevatorStopCom;

/**
 * Add your docs here.
 */
public class ElevatorSub extends Subsystem {

  private WPI_VictorSPX elevatorMotor;
  private DigitalInput elevatorTopLimit;
  private DigitalInput elevatorBottomLimit;
  //private PIDController myPID;
  //private AnalogPotentiometer elevatorPot;

  public ElevatorSub() {
    elevatorMotor = new WPI_VictorSPX(RobotMap.ELEVATOR_MOTOR_CHANNEL);

    elevatorTopLimit = new DigitalInput(RobotMap.UPPER_ELEVATOR_LIMIT_CHANNEL);
    
    elevatorBottomLimit = new DigitalInput(RobotMap.LOWER_ELEVATOR_LIMIT_CHANNEL);
    

    // instantiate pidcontroller and potentiometer
    //elevatorPot = new AnalogPotentiometer(RobotMap.ELEVATOR_POT_CH);
    //myPID = new PIDController(0.1, 1e-4, 150, elevatorPot, elevatorMotor);

    // send pidcontroller and potentiometer to smartdashboard for testing
    //SmartDashboard.putData(elevatorPot);
    //SmartDashboard.putData(myPID);
  }

  public void getElevatorLimits() {
    getTopLimit();
    getBottomLimit();
  }
  
  public boolean getTopLimit() {
    SmartDashboard.putBoolean("Elevator Top Limit", elevatorTopLimit.get());
    return elevatorTopLimit.get();
  }

  public boolean getBottomLimit() {
    SmartDashboard.putBoolean("Elevator Bottom Limit", elevatorBottomLimit.get());
    return elevatorBottomLimit.get();
  }

  public void elevatorUp() {
    elevatorMotor.set(1);
  }

  public void elevatorDown() {
    elevatorMotor.set(-1);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

   /**
   * @return the elevatorPot
   */
  //public double getElevatorPot() {
  //  return elevatorPot.get();
  //}

   /**
    * @param the setpoint for the pid
    */
  //public void setPID(double setPoint) {
   // myPID.setSetpoint(setPoint);
  //}

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorStopCom());
  }
}
