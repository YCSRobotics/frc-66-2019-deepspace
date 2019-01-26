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
public class LiftControl {
    private static TalonSRX liftMotor = new TalonSRX(Constants.kFourBarMotorMaster);

    Joystick operatorController = DriveTrain.operatorController;

    private boolean manualControl = false;
    private double initLiftPosition = 0;

    public LiftControl() {
        //configure sensor boiz
		liftMotor.setSensorPhase(true);
		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		liftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        liftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kP(0, 0.2, 10);
		liftMotor.config_kI(0, 0, 10);
		liftMotor.config_kD(0, 0, 10);
        liftMotor.config_kF(0, 0, 10);

    }

    public void updateLiftTeleop() {
        double liftThrottle = operatorController.getRawAxis(Constants.kLeftYAxis);

        //lift fourbar to position and hold when no more motor output is being applied
        //manual fourbar contrl
        if (Math.abs(liftThrottle) > Constants.kDeadZone) {
            liftMotor.set(ControlMode.PercentOutput, liftThrottle);
            initLiftPosition = getLiftPosition();

            manualControl = true;

        } else {
            liftMotor.set(ControlMode.Position, initLiftPosition);
            manualControl = false;

        }

        //TODO buttons should set position of both fourbar and elevator control
    }

    public static double getLiftPosition() {
        return liftMotor.getSelectedSensorPosition(0);
    }


}
