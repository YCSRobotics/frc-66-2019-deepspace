/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Grizzly Robotics AutoRoutine Class Handles autonomous state machine control
 */
public class AutoRoutine {
    File myFile = new File("CenterCargo.pf1.csv");
    Trajectory trajectory = Pathfinder.readFromCSV(myFile);

    //don't modify the trajectory
    TankModifier modifier = new TankModifier(trajectory).modify(0);

    EncoderFollower leftInput = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower rightInput = new EncoderFollower(modifier.getRightTrajectory());

    public AutoRoutine() {
        leftInput.configureEncoder(SensorData.leftWheelDistance(), 4096, 152.4); //6in to mm 
        rightInput.configureEncoder(SensorData.rightWheelDistance(), 4096, 152.4);

        leftInput.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.kMaxVelocity, 0);
        rightInput.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.kMaxVelocity, 0);
    }

    public void updateAuto() {
        double l = leftInput.calculate(SensorData.leftWheelDistance());
        double r = rightInput.calculate(SensorData.rightWheelDistance());

        double gyro_heading = SensorData.getYaw();
        double desired_heading = Pathfinder.r2d(leftInput.getHeading());  // Should also be in degrees

        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        double turn = 0.8 * (-1.0/80.0) * angleDifference;

        DriveTrain.setMotorOutput(l+turn, r-turn);
    }
    
}
