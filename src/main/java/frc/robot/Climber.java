package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber {
    private static TalonSRX backWenchMotor = new TalonSRX(Constants.kClimberMotor);
    private static TalonSRX frontPullMotor = new TalonSRX(Constants.kPullMotor);

    private static Solenoid climberSolenoid = new Solenoid(Constants.kClimberSolenoid);

    private static Joystick driverJoystick = DriveTrain.driverController;
    private static Joystick operatorJoystick = DriveTrain.operatorController;

    private static final int kFirstWenchPosition = 0;
    private static final int kSecondWenchPosition = 0;
    private static final int kThirdWenchPosition = 0;

    private static final int kFirstElevatorPosition = 0;
    private static final int kSecondElevatorPosition = 0;
    private static final int kThirdElevatorPosition = 0;

    private static double wenchPosition = 0.0;
    private static double elevatorPosition = 0.0;

    private static boolean climberActive = false;

    public Climber() {
        backWenchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        frontPullMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    }

    public void updateClimber() {
        //climber should not be deployed before 30s
        if (DriverStation.getInstance().getMatchTime() > 30) {
            Dashboard.climberReady.setBoolean(false);
            return;

        } else {
            Dashboard.climberReady.setBoolean(true);

        }

        var throttle = driverJoystick.getRawAxis(Constants.kRightYAxis);

        if (operatorJoystick.getRawButton(Constants.kBButton) && !climberActive) {
            climberSolenoid.set(true);

            climberActive = true;

            //disable fourbar output
            FourBarControl.setOverride(true);

            elevatorPosition = ElevatorControl.getLiftPosition();
            wenchPosition = backWenchMotor.getSelectedSensorPosition(0);
        }

        backWenchMotor.set(ControlMode.Position, wenchPosition += 500);
        ElevatorControl.setLiftPosition(elevatorPosition -= 500);

        if (throttle > Constants.kDeadZone) {
            frontPullMotor.set(ControlMode.PercentOutput, throttle);
        }
    }
}
