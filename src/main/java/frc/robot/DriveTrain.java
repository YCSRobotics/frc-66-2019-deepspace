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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private static Timer timer = new Timer();

    private static boolean invertMode = false;
    private static boolean isYawZeroed = false;

    private static double throttleValue;
    private static double turnValue;

    private static double leftOutput;
    private static double rightOutput;

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

        timer.start();
    }

    public void updateDrivetrain(){
        boolean shiftState = driverController.getRawAxis(Constants.kRightTrigger) > 0;
        boolean invertButtonPressed = driverController.getRawButton(Constants.kSelectButton);

        if (shiftState) {
            setSpeedyMode(true);
        } else {
            setSpeedyMode(false);
        }
        
        if ((driverController.getRawButton(Constants.kAButton)) && (!isYawZeroed)) {
            //A-button rising edge, zero sensor and wait one loop for sensor to zero
            SensorData.resetYaw();
            isYawZeroed = true;
        
        } else if (driverController.getRawButton(Constants.kAButton)) {
            //A-button still pressed, sensor now zeroed, calculate drive outputs for Go-Straight
            goStraight();
        
        } else {
            throttleValue = getThrottleInput();//Scaled Throttle Input
            turnValue = getTurnInput();//Scaled Turn Input
            isYawZeroed = false;
        }

        SmartDashboard.putNumber("Joystick Throttle", throttleValue);
        SmartDashboard.putNumber("Turn Value", turnValue);

        calculateMotorOutputs(throttleValue, turnValue);

        //invert mode boiz
        if (invertButtonPressed && timer.get() > 0.5) {
            invertMode = !invertMode;
            timer.reset();

        }

        if (invertMode) {
            //TODO check this code
            leftOutput = -leftOutput;
            rightOutput = -rightOutput;

            var tempVar = leftOutput;

            //switch the sides
            leftOutput = rightOutput;
            rightOutput = tempVar;

        }

        SmartDashboard.putNumber("Left Motor Output", leftOutput);
        SmartDashboard.putNumber("Right Motor Output", rightOutput);

        setMotorOutput(leftOutput, rightOutput);
    }

    private double trim(double input) {
        if (input > 1.0) {
            return -((input - 1.0) * Constants.kSkimGain);

        } else if (input < -1.0) {
            return -((input + 1.0) * Constants.kSkimGain);

        } return 0;

    }

    public static void setSpeedyMode(boolean state) {
        speedyMode.set(state);
    }

    public static double getThrottleInput() {
        //Returns throttle value already scaled for slow mode, elevator up, etc
        double forwardValue = driverController.getRawAxis(Constants.kLeftYAxis);
        boolean speedyMode = driverController.getRawAxis(Constants.kLeftTrigger) > 0;

        if (Math.abs(forwardValue) < Constants.kDeadZone) {
            return 0;
        }
        
        //slow drivetrain when elevator lifted
        if (ElevatorControl.getLiftPosition() > Constants.kElevatorDriveFinesseLimit) {
            forwardValue = forwardValue * Constants.kElevatorDriveMaxSpeed;

        } else if(!speedyMode) {
            forwardValue = forwardValue * Constants.kDriveSpeed;

        }

        return (Math.abs(forwardValue) > Constants.kDeadZone ? -forwardValue : 0.0);
    }

    public static double getTurnInput() {
        //Returns scaled turn input
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);

        if (Math.abs(getThrottleInput()) > Constants.kDeadZone) {
            turnValue = turnValue * Constants.kTurnGain;

        } else {
            turnValue = turnValue * Constants.kTurnFinesseGain;
        }

        return turnValue;
    }

    private void calculateMotorOutputs(double throttle, double turn){
        leftOutput = throttle + turn;
        rightOutput = throttle - turn;

        //apply our skim gains to smooth turning
        leftOutput = leftOutput + trim(rightOutput);
        rightOutput = rightOutput + trim(leftOutput);
    }

    public static void setMotorOutput(double leftMotorValue, double rightMotorValue) {
        leftMaster.set(ControlMode.PercentOutput, leftMotorValue);
        rightMaster.set(ControlMode.PercentOutput, rightMotorValue);
    }

    public static void goStraight() {
        //sets throttle value based on throttle input and turn value based on heading error
        throttleValue = getThrottleInput();
        turnValue = (0 - SensorData.getYaw() ) * Constants.kGyroGain;
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
