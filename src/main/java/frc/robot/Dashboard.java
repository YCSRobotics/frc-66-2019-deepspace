/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Grizzly Robotics Dashboard Class Handles creation of Shuffleboard UI
 */

public class Dashboard {
    private UnitTest unitTest = new UnitTest();

    private ShuffleboardTab driverDisplay = Shuffleboard.getTab("DriverDisplay");

    private ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");
    
    private NetworkTableEntry gyroConnectedKey = diagnosticsTab
                                    .add("Gyro Connected?", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry motorsConnectedKey = diagnosticsTab
                                    .add("CAN Working?", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry cameraTurretKey = diagnosticsTab
                                    .add("Camera Turret Alive?", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry leftWheelDistanceKey = diagnosticsTab
                                    .add("Left Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();    
                                    
    private NetworkTableEntry rightWheelDistanceKey = diagnosticsTab
                                    .add("Right Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    private NetworkTableEntry navYawKey = diagnosticsTab
                                    .add("NavX Yaw", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    private NetworkTableEntry navRollKey = diagnosticsTab
                                    .add("NavX Roll", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();
                                    
    private NetworkTableEntry navPitchKey = diagnosticsTab
                                    .add("NavX Pitch", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry leftMotorPosition = diagnosticsTab
                                    .add("Left Motor Position", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry rightMotorPosition = diagnosticsTab
                                    .add("Right Motor Position", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    public void updateDiagDashboard() {
        gyroConnectedKey.setBoolean(unitTest.navConnected());
        motorsConnectedKey.setBoolean(unitTest.canConnected());
        cameraTurretKey.setBoolean(unitTest.cameraTurretAlive());

        navYawKey.setNumber(SensorData.getYaw());
        navPitchKey.setNumber(SensorData.getPitch());
        navRollKey.setNumber(SensorData.getRoll());

        leftWheelDistanceKey.setNumber(DriveTrain.getLeftWheelDistance());
        rightWheelDistanceKey.setNumber(DriveTrain.getRightWheelDistance());

        leftMotorPosition.setNumber(DriveTrain.getLeftWheelPosition());
        rightMotorPosition.setNumber(DriveTrain.getRightWheelPosition());
    }

    public void updateDriverDashboard() {

    }
  
}
