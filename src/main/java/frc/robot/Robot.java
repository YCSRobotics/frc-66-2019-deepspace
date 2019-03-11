/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default - Manual";
  private static final String kCenterFrontBayRight = "Center Start -> Front Bay Right";
  private static final String kCenterFrontBayLeft = "Center Start -> Front Bay Left";
  private static final String kRgtRocketLvl1 = "Right Rocket Level 1";
  private static final String kLftRocketLvl1 = "Left Rocket Level 1";
  
  private DriveTrain driveTrain = new DriveTrain();
  private Dashboard dashboard = new Dashboard();
  private FourBarControl fourBarControl = new FourBarControl();
  private Intake intake = new Intake();
  private ElevatorControl elevatorControl = new ElevatorControl();
  private CameraTurret turret = new CameraTurret();

  private String m_autonSelected = kDefaultAuto;
  private SendableChooser<String> m_chooser = new SendableChooser<>();

  //called on robot boot
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default - Manual", kDefaultAuto);
    m_chooser.addOption("Center - Ship Right", kCenterFrontBayRight);
    m_chooser.addOption("Center - Ship Left", kCenterFrontBayLeft);

    Dashboard.diagnosticsTab.add(m_chooser).withSize(2,2).withPosition(3,0);
  }
  
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putData("Sandstorm Choices", m_chooser);
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
    m_autonSelected = m_chooser.getSelected();
    
    System.out.println("Auto selected: " + m_autonSelected);

    switch(m_autonSelected){
      case kDefaultAuto:
      default:
      break;
    }


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
