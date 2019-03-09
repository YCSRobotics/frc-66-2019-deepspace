/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

import java.util.Map;

/**
 * Handles camera interactions
 * TODO, move to raspberry pi
 */
public class CameraTurret {
    private static Servo servo = new Servo(0);
    private static Joystick driverJoystick = DriveTrain.driverController;
    private static double currentPosition = 0.6;

    public CameraTurret() {
        UsbCamera cameraServer = CameraServer.getInstance().startAutomaticCapture();
        cameraServer.setResolution(426, 240);
        cameraServer.setFPS(15);

        //add camera to display
        Dashboard.diagnosticsTab.add(cameraServer).withSize(3,4).withProperties(Map.of("Rotation", "QUARTER_CCW"));
        servo.set(currentPosition);

    }

    public void updateCameraTurretTeleop() {
        boolean upwardThrottle = driverJoystick.getRawButton(Constants.kRightBumper);
        boolean downwardThrottle = driverJoystick.getRawButton(Constants.kLeftBumper);

        if (upwardThrottle && currentPosition <= 1.0) {
            currentPosition += 0.02;
        } else if (downwardThrottle && currentPosition >= 0) {
            currentPosition -= 0.02;
        }


        servo.set(currentPosition);

    }

}
