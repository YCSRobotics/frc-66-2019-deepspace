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

/**
 * Handles camera interactions
 * TODO, move to raspberry pi
 */
public class CameraTurret {
    private static Servo servo = new Servo(0);
    private static Joystick driverJoystick = DriveTrain.driverController;
    private static double currentPosition = -1.0;

    public CameraTurret() {
        UsbCamera cameraServer = CameraServer.getInstance().startAutomaticCapture();
        cameraServer.setResolution(640, 480);

        //add camera to display
        Dashboard.driverDisplay.add(cameraServer).withSize(3,3);

    }

    public void updateCameraTurretTeleop() {
        boolean upwardThrottle = driverJoystick.getPOV() == 0;
        boolean downwardThrottle = driverJoystick.getPOV() == 180;

        if (upwardThrottle) {
            currentPosition += 0.1;
        } else if (downwardThrottle) {
            currentPosition -= 0.1;
        }

        servo.set(currentPosition);

    }

    public static boolean cameraTurretAlive() {
        return servo.isAlive();
    }
}
