/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveTrainCommands.MecanumDriveCom;
import frc.robot.subsystems.BuffaloNoseSub;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HerderArmSub;
import frc.robot.subsystems.HerderSub;
import frc.robot.subsystems.LifterDriveSub;
import frc.robot.subsystems.LifterPinSub;
import frc.robot.subsystems.LifterSub;
import frc.robot.subsystems.ServoSub;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveTrainSub driveTrainSub;
  public static BuffaloNoseSub buffaloNoseSub;
  public static ElevatorSub elevatorSub;
  public static LifterSub lifterSub;
  public static LifterDriveSub lifterDriveSub;
  public static LifterPinSub lifterPinSub;
  public static HerderSub herderSub;
  public static HerderArmSub herderArmSub;
  public static ServoSub servoSub;
  public static CameraSub cameraSub;
  public static OI oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrainSub = new DriveTrainSub();
    buffaloNoseSub = new BuffaloNoseSub();
    elevatorSub = new ElevatorSub();
    lifterSub = new LifterSub();
    lifterDriveSub = new LifterDriveSub();
    lifterPinSub = new LifterPinSub();
    herderSub = new HerderSub();
    herderArmSub = new HerderArmSub();
    servoSub = new ServoSub();
    cameraSub = new CameraSub();
    oi = new OI();

    m_chooser.setDefaultOption("Default Auto", new MecanumDriveCom());
    m_chooser.addOption("My Auto", new MecanumDriveCom());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    Robot.herderArmSub.getHerderArmPot();
    Robot.elevatorSub.getTopLimit();
    Robot.elevatorSub.getBottomLimit();
    Robot.cameraSub.getLimelight();

    double FL = driveTrainSub.getCurrent("FL");
    double FR = driveTrainSub.getCurrent("FR");
    double RL = driveTrainSub.getCurrent("RL");
    double RR = driveTrainSub.getCurrent("RR");
    SmartDashboard.putNumber("Current FR", FR);
    SmartDashboard.putNumber("Current FL", FL);
    SmartDashboard.putNumber("Current RR", RR);
    SmartDashboard.putNumber("Current RL", RL);
    SmartDashboard.putNumber("Current All", FL + FR + RL + RR);
    //Timer.delay(0.1); //limit update to every 100ms
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
