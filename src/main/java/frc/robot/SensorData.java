/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;

/**
 * Grizzly Robotics Sensor Data Class
 * Contains static references to grab sensor data
 */
public class SensorData {
    private static AHRS navSensor = new AHRS(SPI.Port.kMXP);
    private static DigitalInput bannerSensor = new DigitalInput(2);

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("ChickenVision");

    private static final double kThirdThreshold = 5;
    private static final double kSecondThreshold = 10;
    private static final double kFirstThreshold = 14;

    public static void resetYaw() { navSensor.reset(); }

    public static double getYaw() {
        return navSensor.getYaw();
    }

    public static double getPitch() {
        return navSensor.getPitch();
    }

    public static double getRoll() {
        return navSensor.getRoll();
    }

    public static boolean getBallSensorState() { return bannerSensor.get(); }

    //TODO apply angle offset
    public static double angleToVisionTarget() {
        double offset = 0;

        double currentDistance = distanceToVisionTarget();

        if (currentDistance >= 120) {
            offset = kThirdThreshold;
        } else if( currentDistance >= 80) {
            offset = kSecondThreshold;
        } else {
            offset = kFirstThreshold;
        }

        double data = table.getEntry("tapeYaw").getDouble(0.0);

        data = data > 0 ? data + offset : data - offset;

        return data;
    }

    public static double distanceToVisionTarget() { return table.getEntry("tapeDistance").getDouble(0.0); }

    public static boolean tapeDetected() { return table.getEntry("tapeDetected").getBoolean(false); }
}

