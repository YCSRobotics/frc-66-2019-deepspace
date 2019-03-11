package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class AutoRoutine {
	//Timer for timed delays
	private Timer timer = new Timer();
	
	//Autonomous Routines
	final static int DO_NOTHING           = 0;
	//Center Start

	//Left Start

	//Right Start
	

	//Autonomous States
    final static int START        			       = 0;
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
		case STOP:
		default:
			stateActionStop();
		}
	}


	private void stateActionStart(){
		
		if(selectedAutonRoutine != DO_NOTHING){
			//TODO:Add Start State code here
		}
		else{
			currentAutonState = STOP;
		}
	}
	
	private void stateActionStop(){
		//Drivetrain.setMoveDistance(0.0, 0.0);
	}
	
	private void setAutonDelay(double delay){
		alarmTime = timer.get() + delay;
	}
}
