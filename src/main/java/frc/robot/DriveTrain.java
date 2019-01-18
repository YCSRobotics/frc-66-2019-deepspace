/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/**
 * Grizzly Robotics Drivetrain File
 * robot movment 
 */
public class DriveTrain {

    TalonSRX leftMaster = new TalonSRX(0);
    TalonSRX leftFollower = new TalonSRX(1);
    TalonSRX rightMaster = new TalonSRX(3);
    TalonSRX rightFollower = new TalonSRX(4);



}
