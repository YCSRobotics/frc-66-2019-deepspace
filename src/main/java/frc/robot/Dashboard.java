/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Grizzly Robotics Dashboard Class Handles creation of Shuffleboard UI
 */

public class Dashboard {
    private UnitTest unitTest = new UnitTest();

    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");
    NetworkTableEntry gyroConnected = diagnosticsTab
                                    .add("Gyro Connected?", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    NetworkTableEntry motorsConnected = diagnosticsTab
                                    .add("CAN Working?", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    NetworkTableEntry leftWheelDistance = diagnosticsTab
                                    .add("Left Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();    
                                    
    NetworkTableEntry rightWheelDistance = diagnosticsTab
                                    .add("Right Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    NetworkTableEntry navYaw = diagnosticsTab
                                    .add("NavX Yaw", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    NetworkTableEntry navRoll = diagnosticsTab
                                    .add("NavX Roll", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();
                                    
    NetworkTableEntry navPitch = diagnosticsTab
                                    .add("NavX Pitch", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();    

    public void updateDashboard() {
        gyroConnected.setBoolean(unitTest.navConnected());
        motorsConnected.setBoolean(unitTest.canConnected());

        navYaw.setNumber(SensorData.getYaw());
        navPitch.setNumber(SensorData.getPitch());
        navRoll.setNumber(SensorData.getRoll());
    }
  
}
