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
	final static int AUTO_DELAY                   = 5;
	final static int STOP						  = 255;

	public static double alarmTime;

	public static int selectedAutonRoutine;
	public static int currentAutonState = START;

	private boolean delaySet = false;

	public AutoRoutine(){

	}
	
	public void setSelectedAutonRoutine(int routine){
		selectedAutonRoutine = routine;
	}
	
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
            case AUTO_DELAY:
                stateAutoDelay();
                break;
            case STOP:
            default:
                stateActionStop();
            }
	}


	private void stateActionStart(){
		if(selectedAutonRoutine != DO_NOTHING){
			if((selectedAutonRoutine==CENTER_LEFT)||(selectedAutonRoutine==CENTER_RIGHT)){
				DriveTrain.setMoveDistance(100.0, 0.3);
				currentAutonState = MOVE_DISTANCE;
			} else if(selectedAutonRoutine==LEFT_ROCKET){
				DriveTrain.setMoveDistance(30.0, 0.3);
				currentAutonState = MOVE_DISTANCE;
			} else if(selectedAutonRoutine==RIGHT_ROCKET){
				DriveTrain.setMoveDistance(30.0, 0.3);
				currentAutonState = MOVE_DISTANCE;
			} else{
				//Should never get here
				currentAutonState = STOP;
			}
		}
		else{
			currentAutonState = STOP;
		}
	}

	private void stateActionMoveDistance(){
		if(!DriveTrain.isMovingDistance()){
		    if (selectedAutonRoutine == RIGHT_ROCKET) {
		        DriveTrain.setTurnToTarget(0.3, 30);
                currentAutonState = TURN_RIGHT;

            } else if(selectedAutonRoutine == LEFT_ROCKET) {
		        DriveTrain.setTurnToTarget(-0.3, 30);
                currentAutonState = TURN_LEFT;

            }else {
                currentAutonState = STOP;
            }

        } else{
			//Wait for move to complete
		}
	}

	private void stateActionTurn() {
	    if(!DriveTrain.isTurning()) {
	        if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
	            DriveTrain.moveToVisionTarget(0.3);
	            currentAutonState = AUTO_DELAY;
	            System.out.println("Moving to Vision Target");
            } else {
	            currentAutonState = STOP;

            }
        } else {
	        //wait for turn to complete
        }
    }

	private void stateActionMoveVisionTarget() {
        if (!DriveTrain.isFollowingTarget()) {
            currentAutonState = STOP;
        } else {
            //Waiting to finish trackingTarget
        }
    }

    private void stateAutoDelay() {
	    if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
            if (!delaySet) {
                timer.reset();
                timer.start();
            }

            //wait half a second
            if (timer.get() > 0.5) {
                selectedAutonRoutine = MOVE_VISION_TARGET;
            }
        }

    }
	
	private void stateActionStop(){
		DriveTrain.setMoveDistance(0.0, 0.0);
	}
	
}
