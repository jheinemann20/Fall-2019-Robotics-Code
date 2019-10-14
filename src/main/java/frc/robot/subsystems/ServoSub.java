/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class ServoSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Servo traverseServo;
  private Servo yawServo;

  // private final double MAX_ANGLE_LEFT = 180.0;
  // private final double MAX_ANGLE_RIGHT = 180.0;
  // private final double MIN_ANGLE_LEFT = 0.0;
  // private final double MIN_ANGLE_RIGHT = 0.0;
  // private final boolean invertLeft = true;
  // private final boolean invertRight = false;
  

  public ServoSub() {
    traverseServo = new Servo(RobotMap.TRAVERSE_SERVO_CHANNEL);
    yawServo = new Servo(RobotMap.YAW_SERVO_CHANNEL);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setTraverseServoAngle(double angle) {
    traverseServo.setAngle(angle);
  }

  public void getTraverseServoAngle() {
    traverseServo.getAngle();
  }

  public void setYawServoAngle(double angle) {
    yawServo.setAngle(angle);
  }

  public void getYawServoAngle() {
    yawServo.getAngle();
  }
}
