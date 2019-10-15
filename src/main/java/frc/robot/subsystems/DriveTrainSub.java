/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTrainCommands.MecanumDriveCom;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrainSub extends Subsystem {
  private CANSparkMax frontLeft, frontRight, rearLeft, rearRight;
  private CANEncoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
  private MecanumDrive mecDrive;
  private DifferentialDrive arcDrive;
  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;
  private DoubleSolenoid driveSol;
  private Joystick myJoy;

  private double deadband;

  public DriveTrainSub() {
    frontLeft = new CANSparkMax(RobotMap.FRONT_LEFT_CHANNEL, MotorType.kBrushless);
    frontRight = new CANSparkMax(RobotMap.FRONT_RIGHT_CHANNEL, MotorType.kBrushless);
    rearLeft = new CANSparkMax(RobotMap.REAR_LEFT_CHANNEL, MotorType.kBrushless);
    rearRight = new CANSparkMax(RobotMap.REAR_RIGHT_CHANNEL, MotorType.kBrushless);

    frontLeftEncoder = frontLeft.getEncoder();
    frontRightEncoder = frontRight.getEncoder();
    backLeftEncoder = rearLeft.getEncoder();
    backRightEncoder = rearRight.getEncoder();

    myJoy = new Joystick(0);

    //frontLeft.setSmartCurrentLimit(40);
    //frontRight.setSmartCurrentLimit(40);
    //rearLeft.setSmartCurrentLimit(40);
    //rearRight.setSmartCurrentLimit(40);
    


    //encoders[0] = frontLeftEncoder;
    //encoders[1] = frontRightEncoder;
    //encoders[2] = backLeftEncoder;
    //encoders[3] = backRightEncoder;

    mecDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    mecDrive.setSafetyEnabled(false);

    leftSide = new SpeedControllerGroup(frontLeft, rearLeft);
    rightSide = new SpeedControllerGroup(frontRight, rearRight);
    arcDrive = new DifferentialDrive(leftSide, rightSide);

    
    frontLeft.setOpenLoopRampRate(0);
    frontRight.setOpenLoopRampRate(0);
    rearLeft.setOpenLoopRampRate(0);
    rearRight.setOpenLoopRampRate(0);

    frontLeft.setIdleMode(IdleMode.kCoast);
    frontRight.setIdleMode(IdleMode.kCoast);
    rearLeft.setIdleMode(IdleMode.kCoast);
    rearRight.setIdleMode(IdleMode.kCoast);
    
    deadband = RobotMap.DEADBAND_CH;

    driveSol = new DoubleSolenoid(RobotMap.DRIVE_SOL_FORWARD_CH, RobotMap.DRIVE_SOL_REVERSE_CH);
  }

  /** 

  public CANEncoder getEncoders() {
    for(int i = 0; i < encoders.length; i++) {
      return encoders[i];
    }
    return dummyEncoder;
  }
  */
  
  public void mecanumDrive(double ySpeed, double xSpeed, double zRotation) {
    if (!myJoy.getRawButton(RobotMap.SLOW_BUTTON_CH))
      mecDrive.driveCartesian(-(addDeadband(ySpeed)), (addDeadband(xSpeed)), -(addDeadband(zRotation)));
    else // Bypasses deadband and slows down movement while the button is held down
      mecDrive.driveCartesian(-(ySpeed * 0.25), (xSpeed * 0.25), -(zRotation * 0.25));
    driveSol.set(DoubleSolenoid.Value.kForward);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    if (!myJoy.getRawButton(RobotMap.SLOW_BUTTON_CH))
      arcDrive.arcadeDrive((addDeadband(-xSpeed)), -(addDeadband(zRotation)));
    else // Bypasses deadband and slows down movement while the button is held down
      arcDrive.arcadeDrive(-(xSpeed * 0.25), -(zRotation * 0.25));
    driveSol.set(DoubleSolenoid.Value.kReverse);
  }

  // Adds deadband to a given axis (for driving only)
  public double addDeadband(double x) {
    if (x >= deadband)
      return x;
    else if (x <= -deadband)
      return x;
    else
      return 0;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MecanumDriveCom());
  }
}
