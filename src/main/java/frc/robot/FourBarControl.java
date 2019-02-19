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
 * Handles controlling the robot fourbar
 */
public class FourBarControl {
    private static TalonSRX fourBarMotorMaster = new TalonSRX(Constants.kFourBarMotor);

    private Joystick operatorController = DriveTrain.operatorController;

    private boolean manualControl = false;
    private double fourBarPosition = 0.0;

    public FourBarControl() {
        //configure sensor boiz
		fourBarMotorMaster.setSensorPhase(true);
		fourBarMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		fourBarMotorMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        fourBarMotorMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        fourBarMotorMaster.selectProfileSlot(0, 0);
		fourBarMotorMaster.config_kP(0, 0.2, 10);
		fourBarMotorMaster.config_kI(0, 0, 10);
		fourBarMotorMaster.config_kD(0, 0, 10);
        fourBarMotorMaster.config_kF(0, 0, 10);

    }

    public void updateFourBarTeleop() {
        double fourBarThrottle = operatorController.getRawAxis(Constants.kLeftYAxis);

        //lift fourbar to position and hold when no more motor output is being applied
        //manual fourbar contrl
        if (Math.abs(fourBarThrottle) > Constants.kDeadZone) {
            fourBarPosition = getFourBarPosition();
            fourBarMotorMaster.set(ControlMode.PercentOutput, fourBarThrottle);

            manualControl = true;

        } else {
            System.out.println("Grabbed Position: "  + fourBarPosition);
            fourBarMotorMaster.set(ControlMode.Position, fourBarPosition);
            manualControl = false;

        }

        //TODO buttons should set position of both fourbar and elevator control
    }

    public static double getFourBarPosition() {
        return fourBarMotorMaster.getSelectedSensorPosition(0);
    }


}
