/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.sun.org.apache.xpath.internal.objects.XBoolean;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/**
 * Grizzly Robotics Drivetrain File
 * robot movment
 */
public class DriveTrain {

    private static WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.kLeftMotorMaster);
    private static WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.kLeftMotorFollower);
    private static WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.kRightMotorMaster);
    private static WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.kRightMoterFollower);

    public static Joystick driverController = new Joystick(Constants.kDriverController);
    public static Joystick operatorController = new Joystick(Constants.kOperatorController);

    private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

    private static double integral = 0; 

    public DriveTrain(){
        //set brake mode
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);

        rightMaster.setInverted(Constants.kInvertRightMotor);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightFollower.getDeviceID());

        leftMaster.setSensorPhase(true);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    }

    public void updateDrivetrain(){
        double forwardValue = driverController.getRawAxis(Constants.kRightYAxis);
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);
        boolean isQuickTurn = driverController.getRawButton(Constants.kAButton);
        
        if (Math.abs(forwardValue) > Constants.kDeadZone) {
            drive.curvatureDrive(forwardValue, turnValue, isQuickTurn);

        } else {
            drive.curvatureDrive(0, 0, isQuickTurn);

        }

    }

    public static void setMotorThrottle(double motorThrottle, double motorTurn) {
        double leftOutput = motorThrottle + motorTurn;
        double rightOutput = motorThrottle - motorTurn;

        leftMaster.set(ControlMode.PercentOutput, leftOutput);
        rightMaster.set(ControlMode.PercentOutput, rightOutput);

    }

    public static void setMotorOutput(double leftMotorValue, double rightMotorValue) {
        leftMaster.set(ControlMode.PercentOutput, leftMotorValue);
        rightMaster.set(ControlMode.PercentOutput, rightMotorValue);
    }

    public static void setGoStraight(int distanceInInches) {
        double distanceError = distanceInInches - getAverageDistance();

        integral += distanceError * Constants.kRobotRate;

        //uses black magic to be awesome
        double motorThrottle = Constants.kDistanceP * distanceError + Constants.kDistanceI + integral;

        //multiple angle by gain (smoothing out the curve) and invert
        double motorTurn = -1 * (SensorData.getYaw() * Constants.kGyroGain);

        setMotorOutput(motorThrottle, motorTurn);

    }

    public static boolean motorTempSuccess() {
        return (leftMaster.getTemperature() == 0 || leftFollower.getTemperature() == 0 || rightMaster.getTemperature() == 0 || rightFollower.getTemperature() == 0);

    }

    public static double getAverageDistance() {
        return (getLeftWheelDistance() + getRightWheelDistance()) / 2;
    }

    public static void resetLeftWheelEncoder() {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public static void resetRightWheelEncoder() {
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public static int getLeftWheelPosition() {
        return leftMaster.getSelectedSensorPosition();
    }

    public static int getRightWheelPosition() {
        return rightMaster.getSelectedSensorPosition();
    }

    public static final double getLeftWheelDistance() {
        return leftMaster.getSelectedSensorPosition() * Constants.kMagMultiplier;
    }

    public static final double getRightWheelDistance() {
        return rightMaster.getSelectedSensorPosition() * Constants.kMagMultiplier;
    }

}
