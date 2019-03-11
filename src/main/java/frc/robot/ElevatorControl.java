/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Grizzly Robotics Lift Class
 * Handles controlling the robot lift
 */
public class ElevatorControl {
    private static TalonSRX liftMotor = new TalonSRX(Constants.kElevatorMotor);

    private Joystick operatorController = DriveTrain.operatorController;

    private double liftPosition = 0.0;
    private double setElevatorPosition = 0.0;
    
    private boolean isOffsetPressed = false;

    public ElevatorControl() {
        //configure sensor boiz
		liftMotor.setSensorPhase(true);
		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		liftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        liftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        //Zero sensor position on initialization
        liftMotor.setSelectedSensorPosition(0);

        liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kP(0, 1.0, 10);
		liftMotor.config_kI(0, 0, 10);
		liftMotor.config_kD(0, 0, 10);
        liftMotor.config_kF(0, 0, 10);

        liftMotor.configOpenloopRamp(Constants.kElevatorOpenRamp);
        liftMotor.configClosedloopRamp(Constants.kElevatorClosedRamp);

    }

    public void updateLiftTeleop() {
        double liftThrottle = -operatorController.getRawAxis(Constants.kLeftYAxis);
        liftPosition = getLiftPosition();

        //update lift to specified position
        if (Math.abs(liftThrottle) > Constants.kDeadZone) {
            liftMotor.set(ControlMode.PercentOutput, liftThrottle);
            isOffsetPressed = false;
            setElevatorPosition = liftPosition;

        } else if (operatorController.getRawButton(Constants.kRightBumper)) {
            if(!isOffsetPressed){
                isOffsetPressed = true;
                setElevatorPosition = liftPosition - Constants.kElevatorPosOffset;
                liftMotor.set(ControlMode.Position, setElevatorPosition);

            }

        } else if (operatorController.getRawButton(Constants.kAButton)) {
            setElevatorPosition = Constants.kElevatorPos1;
            isOffsetPressed = false;
            liftMotor.set(ControlMode.Position, setElevatorPosition);

        } else if (operatorController.getRawButton(Constants.kBButton)) {
            setElevatorPosition = Constants.kElevatorPos2;
            isOffsetPressed = false;
            liftMotor.set(ControlMode.Position, setElevatorPosition);

        } else if (operatorController.getRawButton(Constants.kYButton)) {
            setElevatorPosition = Constants.kElevatorPos3;
            isOffsetPressed = false;
            liftMotor.set(ControlMode.Position, setElevatorPosition);

        } else {
            liftMotor.set(ControlMode.Position, setElevatorPosition);
            isOffsetPressed = false;
        }

    }

    public static double getLiftPosition() {
        return liftMotor.getSelectedSensorPosition(0);
    }


}
