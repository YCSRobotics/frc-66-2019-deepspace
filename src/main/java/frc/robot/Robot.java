/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  
  private DriveTrain driveTrain = new DriveTrain();
  private Dashboard dashboard = new Dashboard();
  private FourBarControl fourBarControl = new FourBarControl();
  private Intake intake = new Intake();
  private ElevatorControl elevatorControl = new ElevatorControl();

  //called on robot boot
  @Override
  public void robotInit() {

  }

  //called every 20ms regardless of game state, after robot init
  @Override
  public void robotPeriodic() {
    dashboard.updateDiagDashboard();
    dashboard.updateDriverDashboard();

  }

  //called at the beginning of auton
  @Override
  public void autonomousInit() {

  }

  //called every 20ms during auton, after auto init
  @Override
  public void autonomousPeriodic() {
    driveTrain.updateDrivetrain();
    fourBarControl.updateFourBarTeleop();
    elevatorControl.updateLiftTeleop();
    intake.updateIntake();

  }

  //called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
    driveTrain.updateDrivetrain();
    fourBarControl.updateFourBarTeleop();
    elevatorControl.updateLiftTeleop();
    intake.updateIntake();

  }

  //called every 20ms during test mode
  @Override
  public void testPeriodic() {

  }
}
