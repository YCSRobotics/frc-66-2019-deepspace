/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
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

    private static Encoder leftWheelEncoder = new Encoder(Constants.kLeftEncoderChannelA, 
                                                            Constants.kLeftEncoderChannelB,
                                                            Constants.kLeftEncoderReversed,
                                                            CounterBase.EncodingType.k4X);

    private static Encoder rightWheelEncoder = new Encoder(Constants.kRightEncoderChannelA, 
                                                            Constants.kRightEncoderChannelB,
                                                            Constants.kRightEncoderReversed,
                                                            CounterBase.EncodingType.k4X);

    private static Joystick driverController = new Joystick(Constants.kDriverController);


    private static double integral = 0; 

    public DriveTrain(){
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightFollower.getDeviceID());

        leftWheelEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
        rightWheelEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

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

    public static void setMotorOutput(double motorThrottle, double motorTurn) {
        double leftOutput = motorThrottle + motorTurn;
        double rightOutput = motorThrottle - motorTurn;

        leftMaster.set(ControlMode.PercentOutput, leftOutput);
        rightMaster.set(ControlMode.PercentOutput, rightOutput);

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

    public static double getLeftWheelDistance() {
        return leftWheelEncoder.getDistance();
    }

    public static double getRightWheelDistance() {
        return rightWheelEncoder.getDistance();
    }

    public static double getAverageDistance() {
        return (getLeftWheelDistance() + getRightWheelDistance()) / 2;
    }


}
