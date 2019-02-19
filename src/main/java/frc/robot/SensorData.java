/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;

/**
 * Grizzly Robotics Sensor Data Class
 * Contains static references to grab sensor data
 */
public class SensorData {
    private static AHRS navSensor = new AHRS(SPI.Port.kMXP);
    private static DigitalInput bannerSensor = new DigitalInput(2);

    public static double getHorizontalAngle() {
        return navSensor.getAngle();
    }

    public static double getYaw() {
        return navSensor.getYaw();
    }

    public static double getPitch() {
        return navSensor.getPitch();
    }

    public static double getRoll() {
        return navSensor.getRoll();
    }

    public static void resetNavYaw() {
        navSensor.reset();
    }

    public static boolean isNavConnected() {
        return navSensor.isConnected();
    }

    public static void resetLeftWheelEncoder() {
        DriveTrain.resetLeftWheelEncoder();
    }

    public static void resetRightWheelEncoder() {
        DriveTrain.resetRightWheelEncoder();
    }

    public static boolean getBallSensorState() { return bannerSensor.get(); }

}
