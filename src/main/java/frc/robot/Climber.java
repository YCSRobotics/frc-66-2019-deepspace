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

    private static final int kElevatorClimbOffset = 1000;

    private static int backDriveDistance = 14000;
    private static final int kBackDriveDistanceSecond = 7000;

    private static final int kDeployRange = 600;

    private static double wenchPosition = 0.0;
    private static double elevatorPosition = 0.0;

    private static boolean hasDeployed = false;
    private static boolean stopElevator = false;

    private static boolean climberActive = false;
    private static boolean wenchDisabled = false;

    //rates for level three
    private static double kClimberIncrementValueLevel3 = 50;
    private static double kElevatorIncrementValueLevel3 = 78;

    //rates for level two
    private static double kClimberIncrementValueLevel2 = 50;
    private static double kElevatorIncrementValueLevel2 = 200;

    private static int kClimbMaxPosition = 24500;
    private static final int kClimbMaxPositionSecond = 17000;
    private static final int kClimberDeployPosition = 15000;

    private static double climbValue = 50;
    private static double elevatorValue = 50;

    private static boolean stopElevatorTrigger = false;
    private static double kElevatorBackDistance = 3000;
    private static boolean secondaryClimberDeploy = false;

    private static boolean prevClimbButtonPressed = false;
    private static boolean finalClimbState = false;

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
        if (DriverStation.getInstance().getMatchTime() > 45) {
            Dashboard.climberReady.setBoolean(false);
            return;

        } else {
            Dashboard.climberReady.setBoolean(true);

        }

        var throttle = driverJoystick.getRawAxis(Constants.kLeftYAxis);
        var climbLevelTwo = driverJoystick.getRawButton(Constants.kRightBumper);
        var climbLevelThree = driverJoystick.getRawButton(Constants.kLeftBumper);

        if (Math.abs(throttle) > Constants.kDeadZone) {
            frontPullMotor.set(ControlMode.PercentOutput, -throttle);
            DriveTrain.throttleValue = -(throttle * Constants.kDriveSpeed);
        } else {
            DriveTrain.throttleValue = 0;
            frontPullMotor.set(ControlMode.PercentOutput, 0);
        }

        if ((climbLevelThree || climbLevelTwo)) {
            prevClimbButtonPressed = true;
        } else {
            prevClimbButtonPressed = false;
        }

        //activate climber for level 3
        if (climbLevelThree && !climberActive) {
            climbValue = kClimberIncrementValueLevel3;
            elevatorValue = kElevatorIncrementValueLevel3;
            climberActive = true;

            enableClimber();

            return;

        //activate climber for level 2
        } else if (climbLevelTwo && !climberActive) {
            climbValue = kClimberIncrementValueLevel2;
            elevatorValue = kElevatorIncrementValueLevel2;
            kClimbMaxPosition = kClimbMaxPositionSecond;
            backDriveDistance = kBackDriveDistanceSecond;
            climberActive = true;

            enableClimber();

            return;
        }

        climberSolenoid.set(true);

        if ((backWenchMotor.getSelectedSensorPosition() > wenchPosition - kDeployRange) && (climbLevelTwo || climbLevelThree) && !hasDeployed) {
            secondaryClimberDeploy = true;
            ElevatorControl.setOverride(true);
            enableClimber();
            System.out.println("Enabled override");
            hasDeployed = true;

        } else if (!hasDeployed) {
            backWenchMotor.set(ControlMode.Position, wenchPosition);
            return;

        } else {
            //continue, we have deployed
        }

        SmartDashboard.putNumber("Elevator Current Position: " , ElevatorControl.getLiftPosition());
        SmartDashboard.putNumber("Elevator Set Position", elevatorPosition);

        /* //manual climb control
        if (throttleClimbForward) {
            wenchPosition += climbValue;
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        } else if (throttleClimbBackward) {
            wenchPosition -= climbValue;
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        } else {
            backWenchMotor.set(ControlMode.Position, wenchPosition);
        }*/

        DriveTrain.turnValue = 0;

        if (backWenchMotor.getSelectedSensorPosition() > kClimbMaxPosition - kElevatorClimbOffset) {
            stopElevator = true;
        }

        SmartDashboard.putNumber("Wench Power", wenchPosition);

        if (ElevatorControl.bottomLimit() && !stopElevatorTrigger) {
            elevatorPosition = ElevatorControl.getLiftPosition() + 80;
            stopElevatorTrigger = true;
            stopElevator = true;
        }

        if (driverJoystick.getPOV() == 0 && !wenchDisabled) {
            wenchPosition = wenchPosition - backDriveDistance;
            wenchDisabled = !wenchDisabled;
            elevatorPosition = ElevatorControl.getLiftPosition() + kElevatorBackDistance;
        }

        if ((climbLevelTwo || climbLevelThree) && backWenchMotor.getSelectedSensorPosition() < kClimbMaxPosition) {
            if (hasDeployed) {
                backWenchMotor.set(ControlMode.Position, wenchPosition += climbValue);

                if (!stopElevator) {
                    ElevatorControl.setLiftPosition(elevatorPosition -= elevatorValue);
                } else {
                    ElevatorControl.setLiftPosition(elevatorPosition);
                }
            }

        } else {
            backWenchMotor.set(ControlMode.Position, wenchPosition);
            ElevatorControl.setLiftPosition(elevatorPosition);
        }
    }

    public static double getWenchPosition() {
        return backWenchMotor.getSelectedSensorPosition(0);
    }

    private static void enableClimber() {
        System.out.println("Set override!");

        climberActive = true;

        wenchPosition = kClimberDeployPosition;
        elevatorPosition = ElevatorControl.getLiftPosition();

        //disable robot outputs
        //FourBarControl.setOverride(true);
        //ElevatorControl.setOverride(true);
        DriveTrain.setOverride(true);

    }
}
