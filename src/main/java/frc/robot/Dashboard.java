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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Dashboard Class
 * Handles creation of Shuffleboard UI
 */

public class Dashboard {
    public static ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

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

    private NetworkTableEntry crossBoxSensor = diagnosticsTab
                                    .add("Cross Bar Sensor", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry elevatorPosition = diagnosticsTab
                                    .add("Elevator Position", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry fourBarPosition = diagnosticsTab
                                    .add("Fourbar Position", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    public Dashboard() {
    }

    public void updateDiagDashboard() {
        Shuffleboard.selectTab("Diagnostics");

        navYawKey.setNumber(SensorData.getYaw());
        navPitchKey.setNumber(SensorData.getPitch());
        navRollKey.setNumber(SensorData.getRoll());

        leftWheelDistanceKey.setNumber(DriveTrain.getLeftWheelDistance());
        rightWheelDistanceKey.setNumber(DriveTrain.getRightWheelDistance());

        leftMotorPosition.setNumber(DriveTrain.getLeftWheelPosition());
        rightMotorPosition.setNumber(DriveTrain.getRightWheelPosition());

        crossBoxSensor.setBoolean(SensorData.getBallSensorState());
        elevatorPosition.setNumber(ElevatorControl.getLiftPosition());
        fourBarPosition.setNumber(FourBarControl.getFourBarPosition());

    }

    public void updateDriverDashboard() {
        //leftWheelDistanceKeyDriver.setNumber(DriveTrain.getLeftWheelDistance());
        //rightWheelDistanceKeyDriver.setNumber(DriveTrain.getRightWheelDistance());
    }
  
}
