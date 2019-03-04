/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Lift Class
 * Handles controlling the robot fourbar
 */
public class FourBarControl {
    private static TalonSRX fourBarMotorMaster = new TalonSRX(Constants.kFourBarMotor);

    private Joystick operatorController = DriveTrain.operatorController;

    private boolean manualControl = false;
    private double fourBarPosition = 0.0;
    private double setPosition = 0.0;
    private boolean isOffsetPressed = false;

    public FourBarControl() {
        //configure sensor boiz
		fourBarMotorMaster.setSensorPhase(true);
		fourBarMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		fourBarMotorMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        fourBarMotorMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        //Zero sensor position on initialization
        fourBarMotorMaster.setSelectedSensorPosition(0);

        fourBarMotorMaster.selectProfileSlot(0, 0);
		fourBarMotorMaster.config_kP(0, 4, 10);
		fourBarMotorMaster.config_kI(0, 0, 10);
		fourBarMotorMaster.config_kD(0, 400, 10);
        fourBarMotorMaster.config_kF(0, 0, 10);

        //max outputs
        fourBarMotorMaster.configPeakOutputForward(Constants.kFourBarMaxForward);
        fourBarMotorMaster.configPeakOutputReverse(Constants.kFourBarMaxReverse);

        fourBarMotorMaster.configOpenloopRamp(Constants.kFourBarRamp);
        fourBarMotorMaster.setNeutralMode(NeutralMode.Brake);
    }

    public void updateFourBarTeleop() {
        double fourBarThrottle = -operatorController.getRawAxis(Constants.kRightYAxis);
        SmartDashboard.putNumber("Current Fourbar Output", fourBarMotorMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Operator Fourbar", fourBarThrottle);
        SmartDashboard.putNumber("Four Bar Set Position", fourBarPosition);

        fourBarPosition = getFourBarPosition();

        //implement feedforward depending on power
        if(fourBarThrottle > Constants.kFourBarDeadZone) {
            fourBarMotorMaster.config_kF(0, 1, 10);
        } else if(fourBarThrottle < -Constants.kFourBarDeadZone) {
            fourBarMotorMaster.config_kF(0, 0, 10);
        }

        //lift fourbar to position and hold when no more motor output is being applied
        //manual fourbar contrl
        if (Math.abs(fourBarThrottle) > Constants.kFourBarDeadZone) {
            setPosition = fourBarPosition;

            fourBarMotorMaster.set(ControlMode.PercentOutput, fourBarThrottle);
            isOffsetPressed = false;
            manualControl = true;

        } else if (operatorController.getRawButton(Constants.kRightBumper)) {
            
            if(!isOffsetPressed){
                isOffsetPressed = true;
                setPosition = fourBarPosition - Constants.kFourBarPosOffset;
                fourBarMotorMaster.set(ControlMode.Position, setPosition);
            }
            else{}//Do nothing, only act on rising edge
        }
        else if (operatorController.getRawButton(Constants.kAButton)) {
            isOffsetPressed = false;
            setPosition = Constants.kFourBarPos1;

            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        }
        else if (operatorController.getRawButton(Constants.kBButton)) {
            setPosition = Constants.kFourBarPos2;
            isOffsetPressed = false;

            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        }
        else if (operatorController.getRawButton(Constants.kYButton)) {
            setPosition = Constants.kFourBarPos3;
            isOffsetPressed = false;
            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        } else {
            fourBarMotorMaster.set(ControlMode.Position, setPosition);
            isOffsetPressed = false;
            manualControl = false;
        }

        //TODO buttons should set position of both fourbar and elevator control
    }


    public static double getFourBarPosition() {
        return fourBarMotorMaster.getSelectedSensorPosition(0);
    }


}
