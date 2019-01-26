/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
        


    }

    public void updateDrivetrain(){

        double forwardValue = driverController.getRawAxis(Constants.kRightYAxis);
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);
        
        if (Math.abs(forwardValue) > Constants.kDeadZone) {

        } else {

        }



    }


}
