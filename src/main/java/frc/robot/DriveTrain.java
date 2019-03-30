/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Drivetrain File
 * Robot Movement
 */
public class DriveTrain {
    private static TalonSRX leftMaster = new TalonSRX(Constants.kLeftMotorMaster);

    private static TalonSRX leftFollower = new TalonSRX(Constants.kLeftMotorFollower);

    private static TalonSRX rightFollower = new TalonSRX(Constants.kRightMotorFollower);
    private static TalonSRX rightMaster = new TalonSRX(Constants.kRightMotorMaster);

    public static Joystick driverController = new Joystick(Constants.kDriverController);
    public static Joystick operatorController = new Joystick(Constants.kOperatorController);

    private static Solenoid speedyMode = new Solenoid(Constants.kShifterSolenoid);

    private static Timer timer = new Timer();

    public static boolean autonomousActive = false;

    private static boolean invertMode = false;
    private static boolean isYawZeroed = false;
    
    private static boolean isMovingDistance = false;
    private static boolean isTurning = false;
    private static boolean isFollowingTarget = false;

    private static double throttleValue;
    private static double turnValue;

    private static double targetDistance;
    private static double turnAngle;

    private static double leftOutput;
    private static double rightOutput;

    //Encoder Monitor variables
    private enum encoderFaultState{PASSING,DETECTING,FAULTED};
    private static int leftEncoderFaultCount = 0;
    private static double leftEncoderValuePrev = 0.0;
    private static encoderFaultState leftEncoderFaultState = encoderFaultState.PASSING;
    private static boolean isLeftEncoderValid = true;

    private static int rightEncoderFaultCount = 0;
    private static double rightEncoderValuePrev = 0.0;
    private static encoderFaultState rightEncoderFaultState = encoderFaultState.PASSING;
    private static boolean isRightEncoderValid = true;

    private static boolean isTargetFound = false;

    public DriveTrain(){
        //set brake mode
        rightMaster.setInverted(Constants.kInvertRightMotor);
        rightFollower.setInverted(Constants.kInvertRightMotor);

        leftMaster.setInverted(Constants.kInvertLeftMotor);
        leftFollower.setInverted(Constants.kInvertLeftMotor);

        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightMaster.getDeviceID());

        leftMaster.setSensorPhase(false);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        rightMaster.setSensorPhase(true);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);

        leftMaster.configOpenloopRamp(Constants.kDriveRampRate);
        rightMaster.configOpenloopRamp(Constants.kDriveRampRate);

        timer.start();
    }

    public void updateDrivetrain() {
        boolean shiftState = driverController.getRawAxis(Constants.kRightTrigger) > 0;
        boolean invertButtonPressed = driverController.getRawButton(Constants.kSelectButton);

        SmartDashboard.putNumber("turnValue", turnValue);

        monitorEncoderInputs();

        if (shiftState) {
            setSpeedyMode(true);
        } else {
            setSpeedyMode(false);
        }


        if (((isMovingDistance) || (isTurning) || (isFollowingTarget)) && autonomousActive) {
            driveAutonomous();

        } else if ((driverController.getRawButton(Constants.kAButton)) && (!isYawZeroed)) {
            //A-button rising edge, zero sensor and wait one loop for sensor to zero
            SensorData.resetYaw();
            isYawZeroed = true;

        } else if (driverController.getRawButton(Constants.kAButton)) {
            //go straight since yaw is now zeroed
            goStraight();

        } else if ((driverController.getRawButton(Constants.kBButton)) && (!isTargetFound) && (!isYawZeroed)) {
            //target not found and yaw is not zeroed
            SensorData.resetYaw();
            isYawZeroed = true;

        } else if ((driverController.getRawButton(Constants.kBButton)) && (!isTargetFound)) {
            //no target found, go straight
            goStraight();

        } else if (driverController.getRawButton(Constants.kBButton)) {
            //target is found, go toward it until
            goStraightVisionTarget();

        } else {
            throttleValue = getThrottleInput();//Scaled Throttle Input
            turnValue = getTurnInput();//Scaled Turn Input

            //disable throttle
            if (Math.abs(throttleValue) > Constants.kDeadZone) {
                enableDrivetrainDynamicBraking(false);
            }

            isYawZeroed = false;

            isTargetFound = SensorData.tapeDetected();
        }

        calculateMotorOutputs(throttleValue, turnValue);

        //invert mode boiz
        if (invertButtonPressed && timer.get() > 0.5) {
            invertMode = !invertMode;
            timer.reset();

        }

        if ((invertMode) && (!isMovingDistance) && (!isTurning)) {
            leftOutput = -leftOutput;
            rightOutput = -rightOutput;

            var tempVar = leftOutput;

            //switch the sides
            leftOutput = rightOutput;
            rightOutput = tempVar;

        }

        setMotorOutput(leftOutput, rightOutput);
    }

    private static void goStraightVisionTarget() {
        throttleValue = getThrottleInput();
        enableDrivetrainDynamicBraking(false);

        var targetAngleVision = SensorData.angleToVisionTarget();

        if (!SensorData.tapeDetected()) {
            //stop running autonomous, lost target
            //lost target go straight in manual control
            isFollowingTarget = false;
            AutoRoutine.isWithinTargetRange = false;
            isTargetFound = false;
            return;

        }

        if (SensorData.distanceToVisionTarget() < Constants.kVisionDistanceLimit) {
            //auto -> in range, go straight into target
            //if manual control, trigger goStraightCheck
            isFollowingTarget = false;
            AutoRoutine.isWithinTargetRange = true;
            isTargetFound = false;

        } else {
            turnValue = -((0 - targetAngleVision) * Constants.kGyroGain);
        }

    }

    private double trim(double input) {
        if (input > 1.0) {
            return -((input - 1.0) * Constants.kSkimGain);

        } else if (input < -1.0) {
            return -((input + 1.0) * Constants.kSkimGain);

        } return 0;

    }

    public static void setSpeedyMode(boolean state) {
        speedyMode.set(state);
    }

    public static double getThrottleInput() {
        //Returns throttle value already scaled for slow mode, elevator up, etc
        double forwardValue = driverController.getRawAxis(Constants.kLeftYAxis);
        boolean speedyMode = driverController.getRawAxis(Constants.kLeftTrigger) > 0;

        if (Math.abs(forwardValue) < Constants.kDeadZone) {
            return 0;
        }
        
        //slow drivetrain when elevator lifted
        if (ElevatorControl.getLiftPosition() > Constants.kElevatorDriveFinesseLimit) {
            forwardValue = forwardValue * Constants.kElevatorDriveMaxSpeed;

        } else if(!speedyMode) {
            forwardValue = forwardValue * Constants.kDriveSpeed;

        }

        return (Math.abs(forwardValue) > Constants.kDeadZone ? -forwardValue : 0.0);
    }

    public static double getTurnInput() {
        //Returns scaled turn input
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);

        if (Math.abs(getThrottleInput()) > Constants.kDeadZone) {
            turnValue = turnValue * Constants.kTurnGain;

        } else {
            turnValue = turnValue * Constants.kTurnFinesseGain;
        }

        return turnValue;
    }

    private void calculateMotorOutputs(double throttle, double turn){
        leftOutput = throttle + turn;
        rightOutput = throttle - turn;

        //apply our skim gains to smooth turning
        leftOutput = leftOutput + trim(rightOutput);
        rightOutput = rightOutput + trim(leftOutput);
    }

    public static void setMotorOutput(double leftMotorValue, double rightMotorValue) {
        leftMaster.set(ControlMode.PercentOutput, leftMotorValue);
        rightMaster.set(ControlMode.PercentOutput, rightMotorValue);
    }

    public static void goStraight() {
        //sets throttle value based on throttle input and turn value based on heading error
        enableDrivetrainDynamicBraking(false);
        throttleValue = getThrottleInput();
        turnValue = (0 - SensorData.getYaw() ) * Constants.kGyroGain;
    }

    public static double getAverageDistance() {
        return (getLeftWheelDistance() + getRightWheelDistance()) / 2;
    }

    public static void resetLeftWheelEncoder() {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public static void resetRightWheelEncoder() {
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public static int getLeftWheelPosition() {
        return leftMaster.getSelectedSensorPosition();
    }

    public static int getRightWheelPosition() {
        return rightMaster.getSelectedSensorPosition() * Constants.kInvertRightMotorMultiplier;
    }

    public static double getLeftWheelDistance() {
        return leftMaster.getSelectedSensorPosition() / Constants.kMagMultiplier;
    }

    public static double getRightWheelDistance() {
        return (rightMaster.getSelectedSensorPosition() / Constants.kMagMultiplier) * Constants.kInvertRightMotorMultiplier;
    }

    public static void setMoveDistance(double distance, double throttle){
		
        SensorData.resetYaw();
        
        leftMaster.setSelectedSensorPosition(0, 0, 0);
		rightMaster.setSelectedSensorPosition(0, 0, 0);
		
        
        targetDistance = distance;
		
		enableDrivetrainDynamicBraking(true);
		
		if(Math.abs(targetDistance) > Constants.kTargetDistanceThreshold) {
    		isMovingDistance = true;
    		throttleValue = throttle;
		}
    	else {
    		isMovingDistance = false;
    		throttleValue = 0.0;
    	}
    }
    //owo
    public static void moveToVisionTarget(double throttle) {
        enableDrivetrainDynamicBraking(true);//CL - enable brake to limit coast after turning

        throttleValue = throttle;

        isFollowingTarget = true;
        AutoRoutine.isWithinTargetRange = false;

    }

    public static void setTurnToTarget(double turn_power, double angle){
        SensorData.resetYaw();
        
        isTurning = true;
		turnAngle = Math.abs(angle);//angle must always be positive 
		turnValue = turn_power;//Turn power
	}
    
    public static void driveAutonomous(){
        double distance_error;
		
		if(isMovingDistance){
			//Move distance without tracking vision target
            //distance_error = targetDistance - getAverageDistance();
            distance_error = targetDistance - getLeftWheelDistance();
			
			//Check Distance
			if((targetDistance > 0) && 
			   (distance_error <= Constants.kTargetDistanceThreshold)){
				//Robot has reached target
				throttleValue = 0.0;
				isMovingDistance = false;

			} else if((targetDistance <= 0) && (distance_error >= -Constants.kTargetDistanceThreshold)){
				//Robot has reached target
				throttleValue = 0.0;
				isMovingDistance = false;

			} else {
				//Have not reached target
			}

			turnValue = (0 - SensorData.getYaw() ) * Constants.kGyroGain;

		} else if (isTurning) {
            if (Math.abs(SensorData.getYaw()) >= turnAngle) {
                throttleValue = 0.0;
                turnValue = 0.0;
                isTurning = false;
            } else {
                //Do Nothing while turning
            }

        } else if (isFollowingTarget) {
		    throttleValue = Constants.kVisionPower;
            goStraightVisionTarget();

		} else {
			//No Auton move in progress
			throttleValue = 0.0;
			turnValue = 0.0;
		}
    }
    
    public static boolean isMovingDistance(){
        return(isMovingDistance);
    }

    public static boolean isTurning(){
        return(isTurning);
    }

    public static boolean isFollowingTarget() {
        return (isFollowingTarget);
    }

    public static void enableDrivetrainDynamicBraking(boolean enable){
		if(enable){
			leftMaster.setNeutralMode(NeutralMode.Brake);
			leftFollower.setNeutralMode(NeutralMode.Brake);
			rightMaster.setNeutralMode(NeutralMode.Brake);
			rightFollower.setNeutralMode(NeutralMode.Brake);
		}
		else{
			leftMaster.setNeutralMode(NeutralMode.Coast);
			leftFollower.setNeutralMode(NeutralMode.Coast);
			rightMaster.setNeutralMode(NeutralMode.Coast);
			rightFollower.setNeutralMode(NeutralMode.Coast);
		}
		
    }

    //are we ever going to use this?
    @SuppressWarnings("Duplicates")
    private void monitorEncoderInputs(){
        double left_encoder_value = getLeftWheelDistance();
        double left_encoder_delta;
        double right_encoder_value = getRightWheelDistance();
        double right_encoder_delta;

        //Left Encoder
        if(Math.abs(leftOutput) > Constants.kMotorOutputTreshold){
            left_encoder_delta = Math.abs(left_encoder_value - leftEncoderValuePrev);
            
            //Check if encoder value has changed more than a given delta
            if((left_encoder_delta > Constants.kEncoderDelta) && (leftEncoderFaultState != encoderFaultState.FAULTED)){
                //Encoder value is changing so it's OK, reset monitors
                leftEncoderFaultCount = 0;
                leftEncoderValuePrev = left_encoder_value;
                leftEncoderFaultState = encoderFaultState.PASSING;
                isLeftEncoderValid = true;
            }
            else if(leftEncoderFaultCount < Constants.kEncoderFaultThreshold){
                //Encoder value has not changed but too soon to set fault, just increment fault count
                leftEncoderFaultCount++;
                leftEncoderFaultState = encoderFaultState.DETECTING;
            }
            else {
                //Time to set the fault
                leftEncoderFaultState = encoderFaultState.FAULTED;
                isLeftEncoderValid = false; 
            }

        } else {
            //Motor is off so reset monitor
            leftEncoderFaultCount = 0;
            leftEncoderValuePrev = left_encoder_value;
            leftEncoderFaultState = encoderFaultState.PASSING;
            isLeftEncoderValid = true;
        }

        //Right Encoder
        if(Math.abs(rightOutput) > Constants.kMotorOutputTreshold){
            right_encoder_delta = Math.abs(right_encoder_value - rightEncoderValuePrev);
            
            //Check if encoder value has changed more than a given delta
            if((right_encoder_delta > Constants.kEncoderDelta) && (rightEncoderFaultState != encoderFaultState.FAULTED)){
                //Encoder value is changing so it's OK, reset monitors
                rightEncoderFaultCount = 0;
                rightEncoderValuePrev = right_encoder_value;
                rightEncoderFaultState = encoderFaultState.PASSING;
                isRightEncoderValid = true;
            }
            else if(rightEncoderFaultCount < Constants.kEncoderFaultThreshold){
                //Encoder value has not changed but too soon to set fault, just increment fault count
                rightEncoderFaultCount++;
                rightEncoderFaultState = encoderFaultState.DETECTING;
            }
            else {
                //Time to set the fault
                rightEncoderFaultState = encoderFaultState.FAULTED;
                isRightEncoderValid = false; 
            }
        } else {
            //Motor is off so reset monitor
            rightEncoderFaultCount = 0;
            rightEncoderValuePrev = right_encoder_value;
            rightEncoderFaultState = encoderFaultState.PASSING;
            isRightEncoderValid = true;
        }
    }

}
