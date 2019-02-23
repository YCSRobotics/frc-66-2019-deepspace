/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Grizzly Robotics Drivetrain File
 * Robot Movement
 */
public class DriveTrain {
    private static TalonSRX leftMaster = new TalonSRX(Constants.kLeftMotorMaster);

    private static TalonSRX leftFollower = new TalonSRX(Constants.kLeftMotorFollower);

    private static TalonSRX rightFollower = new TalonSRX(Constants.kRightMotorFollower);
    private static TalonSRX rightMaster = new TalonSRX(Constants.kRightMotorMaster);

    public static Joystick driverController = new Joystick(Constants.kDriverController);
    public static Joystick operatorController = new Joystick(Constants.kOperatorController);

    private static Solenoid speedyMode = new Solenoid(Constants.kShifterSolenoid);

    private static double integral = 0;
    private static boolean invertMode = false;

    public DriveTrain(){
        //set brake mode
        rightMaster.setInverted(Constants.kInvertRightMotor);
        rightFollower.setInverted(Constants.kInvertRightMotor);

        leftMaster.setInverted(Constants.kInvertLeftMotor);
        leftFollower.setInverted(Constants.kInvertLeftMotor);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightMaster.getDeviceID());

        leftMaster.setSensorPhase(false);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        rightMaster.setSensorPhase(true);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);

        leftMaster.configOpenloopRamp(Constants.kDriveRampRate);
        rightMaster.configOpenloopRamp(Constants.kDriveRampRate);

    }

    public void updateDrivetrain(){
        double throttle = getThrottleInput();
        double turn = getTurnInput();
        double leftOutput = throttle + turn;
        double rightOutput = throttle - turn;

        boolean shiftState = driverController.getRawAxis(Constants.kRightTrigger) > 0;
        boolean slowMode = driverController.getRawAxis(Constants.kLeftTrigger) > 0;
        boolean invertButtonPressed = driverController.getRawButton(Constants.kAButton);

        if (shiftState) {
            setSpeedyMode(true);
        } else {
            setSpeedyMode(false);
        }

        //slow mode button
        if(slowMode) {
            leftOutput = leftOutput * Constants.kSlowModeSpeed;
            rightOutput = rightOutput * Constants.kSlowModeSpeed;
        }

        //slow drivetrain when elevator lifted
        if (ElevatorControl.getLiftPosition() > Constants.kElevatorDriveFinesseLimit) {
            leftOutput = leftOutput * Constants.kElevatorDriveMaxSpeed;
            rightOutput = rightOutput * Constants.kElevatorDriveMaxSpeed;
        }

        //invert mode boiz
        if (invertButtonPressed) {
            invertMode = !invertMode;
        }

        if (invertMode) {
            leftOutput = -leftOutput;
            rightOutput = -rightOutput;
        }

        leftMaster.set(ControlMode.PercentOutput, leftOutput);
        rightMaster.set(ControlMode.PercentOutput, rightOutput);

    }

    public static void setSpeedyMode(boolean state) {
        speedyMode.set(state);
    }

    public static double getThrottleInput() {
        double forwardValue = driverController.getRawAxis(Constants.kLeftYAxis);

        //TODO slow mode on left trigger
        boolean slowModeActive = false;

        double throttle = (Math.abs(forwardValue) > Constants.kDeadZone ? -forwardValue : 0.0);
        return slowModeActive ? throttle * Constants.kDriveSlowMaxSpeed : throttle;
    }

    public static double getTurnInput() {
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);

        return(turnValue >= 0 ? (turnValue*turnValue) : -(turnValue*turnValue));
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
        return rightMaster.getSelectedSensorPosition() * Constants.kInvertRightMotorMultiplier;
    }

    public static double getLeftWheelDistance() {
        return leftMaster.getSelectedSensorPosition() / Constants.kMagMultiplier;
    }

    public static double getRightWheelDistance() {
        return (rightMaster.getSelectedSensorPosition() / Constants.kMagMultiplier) * Constants.kInvertRightMotorMultiplier;
    }

}
