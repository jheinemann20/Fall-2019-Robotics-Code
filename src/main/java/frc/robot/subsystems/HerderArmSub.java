/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.HerderCommands.TestHerderArmStopCom;

/**
 * Add your docs here.
 */
public class HerderArmSub extends Subsystem {

  private AnalogInput herderPot;
  private PIDController herderArmPID;

  private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput;

  private WPI_VictorSPX herderArmMotor;
 

  public HerderArmSub() {
    // create new AnalogInput (potentiometer)
    herderPot = new AnalogInput(RobotMap.HERDER_POT_CH);

    // create herder motor (VictorSPX)
    herderArmMotor = new WPI_VictorSPX(RobotMap.HERDER_ARM_MOTOR_CHANNEL);
    

    // create pidcontroller
    herderArmPID = new PIDController(kP, kI, kD, herderPot, herderArmMotor);
    //herderArmPID.enable();

    // create PID coefficients
    kP = 0.3;
    kI = 0;
    kD = 0;
    kF = 0;
    kMaxOutput = 0.4;
    kMinOutput = -0.4;

    // set PID coefficients
    herderArmPID.setP(kP);
    herderArmPID.setI(kI);
    herderArmPID.setD(kD);

    herderArmPID.setF(kF);
    herderArmPID.setOutputRange(kMinOutput, kMaxOutput);

    //Publish PID values to SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);

    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Voltage", 0);

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // get (voltage) of potentiometer
  public void getHerderArmPot() {
    SmartDashboard.putNumber("Herder Arm Pot Voltage", herderPot.getVoltage());
  }

  public void shuffleUpdate() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double voltage = SmartDashboard.getNumber("Set Voltage", 0);

    if((p != kP)) {
      herderArmPID.setP(p);
      kP = p;
    }

    if((i != kI)) {
      herderArmPID.setI(i);
      kI = i;
    }

    if((d != kD)) {
      herderArmPID.setD(d);
      kD = d;
    }

    if((max != kMaxOutput) || (min != kMinOutput)) {
      herderArmPID.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  public void setHerderArmPosition(double voltage) {
    herderArmPID.setSetpoint(voltage);
    SmartDashboard.putNumber("SetPoint", voltage);
  }

  public void armIn() {
    herderArmMotor.set(0.5);
  }

  public void armOut() {
    herderArmMotor.set(-0.5);
  }

  public void armStop() {
    herderArmMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    
    //Use this command for PID arm control
    // setDefaultCommand(new HerderArmInCom());

    //Use this command for manual arm control
    setDefaultCommand(new TestHerderArmStopCom());
  }
}
