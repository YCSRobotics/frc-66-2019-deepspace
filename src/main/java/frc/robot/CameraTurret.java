/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class CameraTurret {
    private static Servo servo = new Servo(0);
    private static boolean manualControl = false;

    public void updateCameraTurretTeleop() {

    }

    public static void setTurretPosition(double position) {
        servo.set(position);
        manualControl = false;

    }

    public static boolean cameraTurretAlive() {
        return servo.isAlive();
    }
}
