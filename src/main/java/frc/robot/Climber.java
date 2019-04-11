package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private static TalonSRX backWenchMotor = new TalonSRX(Constants.kClimberMotor);
    private static TalonSRX frontPullMotor = new TalonSRX(Constants.kPullMotor);

    private static Solenoid climberSolenoid = new Solenoid(Constants.kClimberSolenoid);

    private static Joystick driverJoystick = DriveTrain.driverController;
    private static Joystick operatorJoystick = DriveTrain.operatorController;

    private static final int kClimbMaxPosition = 20000;
    private static final int kClimberDeployPosition = 14000;

    private static double wenchPosition = 0.0;
    private static double elevatorPosition = 0.0;

    private static boolean climberActive = false;

    private static double kClimberIncrementValueLevel3 = 50;
    private static double kElevatorIncrementValueLevel3 = 135;

    private static double kClimberIncrementValueLevel2 = 50;
    private static double kElevatorIncrementValueLevel2 = 50;

    private static double climbValue = 50;
    private static double elevatorValue = 50;

    public Climber() {
        backWenchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        frontPullMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        //Zero sensor position on initialization
        backWenchMotor.setSelectedSensorPosition(0);

        backWenchMotor.setSensorPhase(true);

        backWenchMotor.selectProfileSlot(0, 0);
        backWenchMotor.config_kP(0, 1.2, 10);
        backWenchMotor.config_kI(0, 0, 10);
        backWenchMotor.config_kD(0, 0, 10);
        backWenchMotor.config_kF(0, 0, 10);
    }

    public void updateClimber() {
        //climber should not be deployed before 30s
        /*
        if (DriverStation.getInstance().getMatchTime() > 30) {
            Dashboard.climberReady.setBoolean(false);
            return;

        } else {
            Dashboard.climberReady.setBoolean(true);

        }*/

        var throttle = driverJoystick.getRawAxis(Constants.kRightYAxis);
        var throttleClimbForward = driverJoystick.getRawButton(Constants.kRightBumper);
        var throttleClimbBackward = driverJoystick.getRawButton(Constants.kLeftBumper);

        /*
        //activate climber for level 3
        if (driverJoystick.getRawButton(Constants.kLeftBumper) && !climberActive) {
            climbValue = kClimberIncrementValueLevel3;
            elevatorValue = kElevatorIncrementValueLevel3;

            enableClimber();

        //activate climber for level 2
        } else if (driverJoystick.getRawButton(Constants.kRightBumper) && !climberActive) {
            climbValue = kClimberIncrementValueLevel2;
            elevatorValue = kElevatorIncrementValueLevel2;

            enableClimber();

        }

        if (!climberActive) {
            return;
        }
        */

        SmartDashboard.putNumber("Elevator Current Position: " , ElevatorControl.getLiftPosition());
        SmartDashboard.putNumber("Elevator Set Position", elevatorPosition);

        if (throttleClimbForward) {
            wenchPosition += climbValue;
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        } else if (throttleClimbBackward) {
            wenchPosition -= climbValue;
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        } else {
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        }

        /*
        SmartDashboard.putNumber("Wench Power", wenchPosition);
        if (Math.abs(throttleClimb) > Constants.kDeadZone + 0.3 && backWenchMotor.getSelectedSensorPosition() < kClimbMaxPosition) {
            backWenchMotor.set(ControlMode.Position, wenchPosition += climbValue);

            if (backWenchMotor.getSelectedSensorPosition() < kClimbMaxPosition - 3000) {
                ElevatorControl.setLiftPosition(elevatorPosition -= elevatorValue);
            } else {
                ElevatorControl.setLiftPosition(elevatorPosition);
            }

        } else {
            System.out.println("Setting wench position to: " + wenchPosition);
            backWenchMotor.set(ControlMode.Position, wenchPosition);
            ElevatorControl.setLiftPosition(elevatorPosition);
        }

        if (Math.abs(throttle) > Constants.kDeadZone) {
            frontPullMotor.set(ControlMode.PercentOutput, throttle);
        }
        */
    }

    public static double getWenchPosition() {
        return backWenchMotor.getSelectedSensorPosition(0);
    }

    private static void enableClimber() {
        System.out.println("Set override!");

        climberSolenoid.set(true);
        climberActive = true;

        wenchPosition = kClimberDeployPosition;
        elevatorPosition = ElevatorControl.getLiftPosition();

        //disable robot outputs
        //FourBarControl.setOverride(true);
        //ElevatorControl.setOverride(true);
        //DriveTrain.setOverride(true);

    }
}
