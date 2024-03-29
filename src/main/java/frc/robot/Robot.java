/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Drive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Looper mEnabledLooper = new Looper();
  private Looper mDisabledLooper = new Looper();

  private XboxController mController = new XboxController(Constants.kXboxControllerPort);

  SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private Drive mDrive;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    mDrive = Drive.getInstance();
    mSubsystemManager.setSubsystems(mDrive);

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mSubsystemManager.registerDisabledLoops(mDisabledLooper);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    mDrive.stop();

    mDisabledLooper.stop();
    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double throttle = mController.getY(Hand.kLeft);
    double turn = mController.getX(Hand.kRight);
    if (Math.abs(throttle) < 0.1) throttle = 0;
    if (Math.abs(turn) < 0.1) turn = 0;
    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("turn", turn);
    boolean isQuickTurn = mController.getTriggerAxis(Hand.kRight) > 0.5;
    SmartDashboard.putBoolean("isQuickTurn", isQuickTurn);
    mDrive.setOpenLoop(throttle, turn, isQuickTurn);
  }

  @Override
  public void teleopInit() {
    mDisabledLooper.stop();
    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testInit() {
    mDrive.stop();
  }

  @Override
  public void disabledInit() {
    mDrive.stop();

    mEnabledLooper.stop();
    mDisabledLooper.start();
  }
  
  @Override
  public void disabledPeriodic() {
    mDrive.stop();
  }
}
