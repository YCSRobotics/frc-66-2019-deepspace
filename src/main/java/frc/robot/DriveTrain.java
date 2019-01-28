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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
/**
 * Grizzly Robotics Drivetrain File
 * robot movment
 */
public class DriveTrain {

    private static TalonSRX leftMaster = new TalonSRX(Constants.kLeftMotorMaster);
    private static TalonSRX leftFollower = new TalonSRX(Constants.kLeftMotorFollower);
    private static TalonSRX rightMaster = new TalonSRX(Constants.kRightMotorMaster);
    private static TalonSRX rightFollower = new TalonSRX(Constants.kRightMoterFollower);

    public static Joystick driverController = new Joystick(Constants.kDriverController);
    public static Joystick operatorController = new Joystick(Constants.kOperatorController);

    private static double integral = 0; 

    public DriveTrain(){
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightFollower.getDeviceID());

        leftMaster.setSensorPhase(true);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    }

    public void updateDrivetrain(){

        double forwardValue = driverController.getRawAxis(Constants.kRightYAxis);
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);
        
        double leftValue = forwardValue + turnValue;
        double rightValue = forwardValue - turnValue;

        if (Math.abs(forwardValue) > Constants.kDeadZone) {
            leftMaster.set(ControlMode.PercentOutput, forwardValue);
            rightMaster.set(ControlMode.PercentOutput, forwardValue);

        } else {
            leftMaster.set(ControlMode.PercentOutput, 0.0);
            rightMaster.set(ControlMode.PercentOutput, 0.0);

        }

        if (Math.abs(turnValue) > Constants.kDeadZone) {
            leftMaster.set(ControlMode.PercentOutput, leftValue);
            rightMaster.set(ControlMode.PercentOutput, rightValue);

        } else {
            leftMaster.set(ControlMode.PercentOutput, 0.0);
            rightMaster.set(ControlMode.PercentOutput, 0.0);

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
        return (getLeftWheelPosition() + getRightWheelPosition()) / 2;
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

}
