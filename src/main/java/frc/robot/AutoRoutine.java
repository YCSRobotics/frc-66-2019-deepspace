package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;

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
	final static int STOP						  = 255;

	public static double alarmTime;

	public static int selectedAutonRoutine;
	public static int currentAutonState = START;
	
	public AutoRoutine(){

	}
	
	public void setSelectedAutonRoutine(int routine){
		selectedAutonRoutine = routine;
	}
	
	public void updateAutoRoutine(){
		
		switch(currentAutonState){
		case START:
			stateActionStart();
			break;
		case MOVE_DISTANCE:
			stateActionMoveDistance();
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
				currentAutonState = STOP;
			} else if(selectedAutonRoutine==RIGHT_ROCKET){
				currentAutonState = STOP;
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
			currentAutonState = STOP;
		}
		else{
			//Wait for move to complete
		}
	}
	
	private void stateActionStop(){
		//Drivetrain.setMoveDistance(0.0, 0.0);
	}
	
	private void setAutonDelay(double delay){
		alarmTime = timer.get() + delay;
	}

	
}
