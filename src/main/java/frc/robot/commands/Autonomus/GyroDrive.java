// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// // Drives straight using the gyroscope

// package frc.robot.commands.Autonomus;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;

// public class GyroDrive extends Command {

//   ADXRS450_Gyro myGyro;

//   public GyroDrive() {
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     requires(Robot.driveTrainSub);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//     myGyro = new ADXRS450_Gyro();
//     myGyro.reset();
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     double angle = myGyro.getAngle();
//     if (angle > 180)
//       angle = -(180 - (angle - 180));
//     Robot.driveTrainSub.mecanumDrive(Robot.oi.getY(), 0, -angle / 180);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }
