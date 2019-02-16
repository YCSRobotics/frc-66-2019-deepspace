/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class Intake {
    private final static TalonSRX intakeMotor = new TalonSRX(Constants.kIntakeMotor);
    private Joystick operatorController = DriveTrain.operatorController;

    private static boolean manualControl = true;

    public Intake() {

    }

    public void updateIntake() {
        double intakeIn = operatorController.getRawAxis(Constants.kLeftTrigger);
        double intakeOut = operatorController.getRawAxis(Constants.kRightTrigger);

        if (manualControl) {
            intakeMotor.set(ControlMode.PercentOutput, intakeIn > 0 ? intakeIn : intakeOut > 0 ? intakeOut : 0);
        }

    }

    public static void setIntakeState(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);

    }

    public static void setControlState(boolean manualControlEnabled) {
        manualControl = manualControlEnabled;
    }

}
