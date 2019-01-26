/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

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

    public DriveTrain(){
        


    }

    public void updateDrivetrain(){

        double forwardValue = driverController.getRawAxis(Constants.kRightYAxis);
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);
        
        if (Math.abs(forwardValue) > Constants.kDeadZone) {

        } else {

        }

    }

    public static boolean motorTempSuccess() {
        if (leftMaster.getTemperature() == 0 || leftFollower.getTemperature() == 0 || rightMaster.getTemperature() == 0 || rightFollower.getTemperature() == 0) {
            return false;
            
        } else {
            return true;

        }

    }


}
