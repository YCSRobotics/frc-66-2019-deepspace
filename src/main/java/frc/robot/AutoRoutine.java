package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutine {
	//Timer for timed delays
	private Timer timer = new Timer();
	
	//Autonomous Routines
	final static int DO_NOTHING           = 0;
	final static int CENTER_LEFT		  = 1;
	final static int CENTER_RIGHT		  = 2;
	final static int LEFT_ROCKET		  = 3;
	final static int RIGHT_ROCKET		  = 4;

	//Autonomous States
	final static int START        			      = 0;
	final static int MOVE_DISTANCE				  = 1;
	final static int TURN_LEFT					  = 2;
	final static int TURN_RIGHT					  = 3;
	final static int MOVE_VISION_TARGET           = 4;
	final static int AUTO_TURN_DELAY    		  = 5;
	final static int RELEASE_HATCH                = 6;
	final static int MOVE_DISTANCE_TARGET         = 7;
	final static int BACK_TARGET                  = 8;
	final static int STOP						  = 255;

	public static double alarmTime;

	public static int selectedAutonRoutine;
	public static int currentAutonState = START;
	public static boolean isWithinTargetRange = false;

	private boolean delaySet = false;

	public AutoRoutine(){

	}
	
	public void setSelectedAutonRoutine(int routine){
		selectedAutonRoutine = routine;
	}

	/**
	 * Autonomous State Machine
	 */
	public void updateAutoRoutine(){
        SmartDashboard.putNumber("Current Auto Routine", currentAutonState);
		switch(currentAutonState){
            case START:
                stateActionStart();
                break;
            case MOVE_DISTANCE:
                stateActionMoveDistance();
                break;
            case TURN_LEFT:
                stateActionTurn();
                break;
            case TURN_RIGHT:
                stateActionTurn();
                break;
            case MOVE_VISION_TARGET:
                stateActionMoveVisionTarget();
                break;
            case AUTO_TURN_DELAY:
                stateAutoTurnDelay();
                break;
			case MOVE_DISTANCE_TARGET:
				stateMoveDistanceTarget();
				break;
			case RELEASE_HATCH:
				stateReleaseHatch();
				break;
			case BACK_TARGET:
				stateBackFromTarget();
				break;
            case STOP:
            default:
                stateActionStop();
            }
	}


	private void stateActionStart(){
		if(selectedAutonRoutine != DO_NOTHING){
			if((selectedAutonRoutine==CENTER_LEFT)||(selectedAutonRoutine==CENTER_RIGHT)){
				DriveTrain.setMoveDistance(Constants.kCenterGoStraightInitDistance, Constants.kCenterGoStraightInitPower);
				currentAutonState = MOVE_DISTANCE;

			} else if(selectedAutonRoutine==LEFT_ROCKET){//CL - Can probably combine L/R rocket transitions but we will see
				DriveTrain.setMoveDistance(Constants.kRocketInitDistance, Constants.kRocketInitPower);
				currentAutonState = MOVE_DISTANCE;

			} else if(selectedAutonRoutine==RIGHT_ROCKET){
				DriveTrain.setMoveDistance(Constants.kRocketInitDistance, Constants.kRocketInitPower);
				currentAutonState = MOVE_DISTANCE;

			} else{
				//Should never get here
				currentAutonState = STOP;
			}

		} else{
			currentAutonState = STOP;
		}
	}

	private void stateActionMoveDistance(){
		if(!DriveTrain.isMovingDistance()){
			if((selectedAutonRoutine==CENTER_LEFT)||(selectedAutonRoutine==CENTER_RIGHT)){
				//CL - Added this since not sure if this is final action before stop or not
				DriveTrain.moveToVisionTarget(Constants.kVisionPower);
				currentAutonState = MOVE_VISION_TARGET;

			} else if (selectedAutonRoutine == RIGHT_ROCKET) {
		        DriveTrain.setTurnToTarget(Constants.kRocketTurnPower, Constants.kRocketTurnAngle);
                currentAutonState = TURN_RIGHT;

            } else if(selectedAutonRoutine == LEFT_ROCKET) {
		        DriveTrain.setTurnToTarget(-Constants.kRocketTurnPower, Constants.kRocketTurnAngle);
                currentAutonState = TURN_LEFT;

            } else {
                currentAutonState = STOP;
            }

        } else{
			//Wait for move to complete
		}
	}

	private void stateActionTurn() {
	    if(!DriveTrain.isTurning()) {
	        if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
				//Reached target angle but still turning due to inertia, give time to stop
				setAutonDelay(0.5);
	            currentAutonState = AUTO_TURN_DELAY;

            } else {
	            currentAutonState = STOP;

            }

        } else {
	        //wait for turn to complete

        }
    }

    private void stateMoveDistanceTarget() {
		if (!DriveTrain.isMovingDistance()) {
			currentAutonState = RELEASE_HATCH;

		}
	}

	private void stateBackFromTarget() {
		if(!DriveTrain.isMovingDistance()) {
			currentAutonState = STOP;

		} else {
			//wait to finish backing up

		}
	}

	private void stateReleaseHatch() {
		Intake.setHatchState(true);
		DriveTrain.setMoveDistance(Constants.kBackupDistance, -Constants.kBackupPower);
		currentAutonState = BACK_TARGET;
	}

	private void stateActionMoveVisionTarget() {
		if (selectedAutonRoutine == CENTER_LEFT || selectedAutonRoutine == CENTER_RIGHT) {
			if (!DriveTrain.isFollowingTarget()) {
				if (isWithinTargetRange) {
					currentAutonState = RELEASE_HATCH;
				} else {
					currentAutonState = STOP;
					//TODO dead reckoning place on target
				}
			}

		} else if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
			if (!DriveTrain.isFollowingTarget()) {
				if (isWithinTargetRange) {
					//ram that BOI
					DriveTrain.setMoveDistance(Constants.kRammingDistance, Constants.kRammingPower);
					currentAutonState = MOVE_DISTANCE_TARGET;
				} else {
					//not within range, go forward a little bit
					DriveTrain.setMoveDistance(Constants.kRocketDeadReckoningDistance, Constants.kRocketDeadReckoningPower);
					currentAutonState = STOP;
				}

			} else {
				//Waiting to finish trackingTarget

			}
		}
    }

    private void stateAutoTurnDelay() {
		if(timer.get() >= alarmTime) {
			if((selectedAutonRoutine == LEFT_ROCKET)||(selectedAutonRoutine == RIGHT_ROCKET)){
				DriveTrain.moveToVisionTarget(Constants.kVisionPower);
				currentAutonState = MOVE_VISION_TARGET;
				System.out.println("Moving to Vision Target");

			} else{
				//Should never get here, but if we do Stop
				currentAutonState = STOP;

			}

		} else{
			//Wait for timer to expire
		}

    }
	
	private void stateActionStop(){
		//allow coast after stop
		//DriveTrain.setMoveDistance(0.0, 0.0);
	}

	private void setAutonDelay(double delay){
		//Method to make delay timer generic
		alarmTime = timer.get() + delay;
	}
	
}
