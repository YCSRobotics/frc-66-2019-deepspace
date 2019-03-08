/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.Map;

public class Robot extends TimedRobot {
  
  private DriveTrain driveTrain = new DriveTrain();
  private Dashboard dashboard = new Dashboard();
  private FourBarControl fourBarControl = new FourBarControl();
  private Intake intake = new Intake();
  private ElevatorControl elevatorControl = new ElevatorControl();
  private CameraTurret turret = new CameraTurret();

  //called on robot boot
  @Override
  public void robotInit() {
    HttpCamera cameraServer = new HttpCamera("CoProcessorCamera", Constants.kVisionCam);
    cameraServer.setResolution(426, 240);
    cameraServer.setFPS(15);

    //add camera to display
    Dashboard.diagnosticsTab.add(cameraServer).withSize(3,4).withProperties(Map.of("Rotation", "QUARTER_CCW"));
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
    turret.updateCameraTurretTeleop();

  }

  //called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
    driveTrain.updateDrivetrain();
    fourBarControl.updateFourBarTeleop();
    elevatorControl.updateLiftTeleop();
    intake.updateIntake();
    turret.updateCameraTurretTeleop();

  }

  //called every 20ms during test mode
  @Override
  public void testPeriodic() {

  }
}
