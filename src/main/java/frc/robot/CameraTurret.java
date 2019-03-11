/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

/**
 * Handles camera interactions
 * TODO, move to raspberry pi
 */
public class CameraTurret {
    private static Servo servo = new Servo(0);
    private static Joystick driverJoystick = DriveTrain.driverController;
    private static double currentPosition = 0.6;
    private static boolean secondaryCamera = false;

    private Timer timer = new Timer();

    public CameraTurret() {
        //don't initialize camera server in initialization
        //causes crashes
        if (RobotBase.isSimulation()) {
            return;
        }

        HttpCamera camera = new HttpCamera("PiCam", Constants.kVisionCam);
        CameraServer.getInstance().addCamera(camera);

        //add camera to display
        Dashboard.driverDisplayTab.add(camera).withSize(3,4).withPosition(0,0);

        servo.set(currentPosition);
        timer.reset();
        //timer.start();

    }

    public void updateCameraTurretTeleop() {
        boolean upwardThrottle = driverJoystick.getRawButton(Constants.kRightBumper);
        boolean downwardThrottle = driverJoystick.getRawButton(Constants.kLeftBumper);

        if (upwardThrottle && currentPosition <= 1.0) {
            currentPosition += 0.02;
        } else if (downwardThrottle && currentPosition >= 0) {
            currentPosition -= 0.02;
        }

        /*if (driverJoystick.getRawButton(Constants.kYButton) &&  timer.get() > 0.5) {
            secondaryCamera = !secondaryCamera;

            timer.reset();

            SmartDashboard.putBoolean("Primary Camera?", !secondaryCamera);

            if (secondaryCamera) {
                server.setSource(cameraServer2);
            } else {
                server.setSource(cameraServer);
            }
        }*/

        servo.set(currentPosition);

    }

}
