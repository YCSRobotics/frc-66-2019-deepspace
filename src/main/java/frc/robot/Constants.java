/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



/**
 * Grizzly Robotics Constant File
 * Constant variable names should be in Hungarian notation
 * Example: int kDriveLeftMotorMaster = 0;
 * Hungarian notation variables start with k
 */
public class Constants {
    //motor constants
    public static final int kLeftMotorFollower = 1;
    public static final int kLeftMotorMaster = 0;
    public static final int kRightMotorMaster = 2;
    public static final int kRightMotorFollower = 3;
    public static final int kElevatorMotor = 4;
    public static final int kFourBarMotor = 5;
    public static final int kIntakeMotor = 6;

    public static final boolean kInvertRightMotor = false;
    public static final boolean kInvertLeftMotor = true;

    public static final int kInvertRightMotorMultiplier = -1;

    public static final double kDriveRampRate = 0.08;

    //encoder constants
    public static final int kEncoderDistancePerRevolution = 4096;

    public static final int kWheelDiameter = 6;
    public static final double kEncoderRotationRate = 5.4;

    public static final double kPi = 3.14159265;

    public static final double kMagMultiplier = ((kEncoderDistancePerRevolution*kEncoderRotationRate)/(kPi * kWheelDiameter));

    //joystick constants
    public final static int kDriverController = 0;
    public final static int kOperatorController = 1;

    public static final int kLeftXAxis = 0;
    public static final int kLeftYAxis = 1;
    public static final int kRightXAxis = 4;
    public static final int kRightYAxis = 5;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kAButton = 0;
    public static final int kBButton = 1;
    public static final int kXButton = 2;
    public static final int kYButton = 3;
    public static final int kLeftBumper = 4;
    public static final int kRightBumper = 5;
    public static final int kSelectButton = 6;
    public static final int kStartButton = 7;

    //just a filler, needs to be changed
    public static final double kDeadZone = 0.08;

    //distance PID
    public static final int kDistanceP = 1;
    public static final int kDistanceI = 1;

    //go straight PID
    public static final int kGoStraightP = 1;
    public static final int kGoStraightI = 1;

    //gains
    public static final double kGyroGain = 0.08;

    //global constants
    public static final int kRobotRate = 5; //20ms per loop

    //autonomous constants
    public static final String kVisionCam = "10.0.66.12:1181/stream.mjpg";

    //slow mode max speed
    public static final double kDriveSlowMaxSpeed = 0.5;
    public static final double kIntakeSlowMaxSpeed = 0.1;

    //solenoids
    public static final int kShifterSolenoid = 0;

    //camera turret positions
    public static final double kLowPosition = 0.55;
    public static final double kMidPosition = 0.62;
    public static final double kHighPosition = 0.70;

    //elevator
    public static final double kElevatorDriveFinesseLimit = 20000;
    public static final double kElevatorDriveMaxSpeed = 0.2;
    public static final double kElevatorOpenRamp = 0.3;
    public static final double kElevatorClosedRamp = 0.1;

    //fourbar
    public static final double kFourBarRamp = 0.3;

    //drivetrain
    public static final double kSlowModeSpeed = 0.4;
    public static final double kSkimGain = 0.15;

}
