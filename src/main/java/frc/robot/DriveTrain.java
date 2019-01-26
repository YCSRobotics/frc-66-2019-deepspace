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

import edu.wpi.first.wpilibj.Joystick;
/**
 * Grizzly Robotics Drivetrain File
 * robot movment
 */
public class DriveTrain {

    public static TalonSRX leftMaster = new TalonSRX(Constants.kLeftMotorMaster);
    public static TalonSRX leftFollower = new TalonSRX(Constants.kLeftMotorFollower);
    public static TalonSRX rightMaster = new TalonSRX(Constants.kRightMotorMaster);
    public static TalonSRX rightFollower = new TalonSRX(Constants.kRightMoterFollower);

    public static Joystick driverController = new Joystick(Constants.kDriverController);

    public DriveTrain(){
        leftMaster.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightFollower.getDeviceID());
    }

    public void updateDrivetrain(){

        double forwardValue = driverController.getRawAxis(Constants.kRightYAxis);
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);
        
        double leftValue = forwardValue + turnValue;
        double rightValue = forwardValue - turnValue;

        if(Math.abs(forwardValue) > Constants.kDeadZone){
            leftMaster.set(ControlMode.PercentOutput, forwardValue);
            rightMaster.set(ControlMode.PercentOutput, forwardValue);
        }else{
            leftMaster.set(ControlMode.PercentOutput, 0.0);
            rightMaster.set(ControlMode.PercentOutput, 0.0);
        }

        if(Math.abs(turnValue) > Constants.kDeadZone){
            leftMaster.set(ControlMode.PercentOutput, leftValue);
            rightMaster.set(ControlMode.PercentOutput, rightValue);
        }else{
            leftMaster.set(ControlMode.PercentOutput, 0.0);
            rightMaster.set(ControlMode.PercentOutput, 0.0);
        }


    }


}
