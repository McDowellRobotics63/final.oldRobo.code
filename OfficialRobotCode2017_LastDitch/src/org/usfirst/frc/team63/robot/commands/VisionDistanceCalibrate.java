package org.usfirst.frc.team63.robot.commands;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team63.robot.Kinematics;
import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class VisionDistanceCalibrate extends Command {
	
	private double distance_traveled = 0;	
			
	private double last_front_left_inches = 0;
	private double last_rear_left_inches = 0;
	private double last_front_right_inches = 0;
	private double last_rear_right_inches = 0;
	
	List<Double> debug_list_time = null;
	List<Double> debug_list_dist = null;
	List<Double> debug_list_area = null;
	
	private PrintWriter pw = null;
	
	private final Timer totalTimer;
	
    public VisionDistanceCalibrate() {
    	requires(Robot.vision);
    	//requires(Robot.drive);
    	setInterruptible(false);
    	totalTimer = new Timer();    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("VisionDistanceCalibrate initialize()\n");
    	try {
			pw = new PrintWriter(new File("/home/lvuser/vision_dist_calibrate" + System.currentTimeMillis() + ".csv"));
			debug_list_time = new ArrayList<>();
			debug_list_dist = new ArrayList<>();
			debug_list_area = new ArrayList<>();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
    
    	//Start counting distance from zero
    	Robot.drive.resetEncoders();
    	
    	totalTimer.reset();
    	totalTimer.start();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	//System.out.println("VisionDistanceCalibrate execute()\n");
    	double curr_front_left_inches = Robot.drive.getFrontLeftDistanceInches();
    	double curr_rear_left_inches = Robot.drive.getRearLeftDistanceInches();
    	double curr_front_right_inches = Robot.drive.getFrontRightDistanceInches();
    	double curr_rear_right_inches = Robot.drive.getRearRightDistanceInches();	

    	double delta_dist = 0;
    	
    	RigidTransform2d.Delta delta = Kinematics.forwardKinematics(
    			curr_front_left_inches - this.last_front_left_inches,
    			curr_rear_left_inches - this.last_rear_left_inches,
    			curr_front_right_inches - this.last_front_right_inches,
    			curr_rear_right_inches - this.last_rear_right_inches, 0);
    	
    	last_front_left_inches = curr_front_left_inches;
    	last_rear_left_inches = curr_rear_left_inches;
    	last_front_right_inches = curr_front_right_inches;
    	last_rear_right_inches = curr_rear_right_inches;	
    	
		delta_dist = delta.dy;
    	
    	distance_traveled = distance_traveled + delta_dist;
    	
		debug_list_time.add(totalTimer.get());
		debug_list_dist.add(distance_traveled);
		
    	if(Robot.vision.updateContourData())
    	{
			debug_list_area.add(Robot.vision.getTargetArea());
    	}
    	else
    	{
    		debug_list_area.add(-1.0);
    	}
    	
//		if(Robot.oi.stick1_dpadUp.get())
//		{
//			Robot.drive.moveStraightForwardBack(SmartDashboard.getNumber("vision_test_creep_speed", 0));
//		}
//		else if(Robot.oi.stick1_dpadDown.get())
//		{
//			Robot.drive.moveStraightForwardBack(-SmartDashboard.getNumber("vision_test_creep_speed", 0));
//		}
//		else if(Robot.oi.stick1_dpadLeft.get())
//		{
//			Robot.drive.moveStraightLeftRight(SmartDashboard.getNumber("vision_test_creep_speed", 0));
//		}
//		else if(Robot.oi.stick1_dpadRight.get())
//		{
//			Robot.drive.moveStraightLeftRight(-SmartDashboard.getNumber("vision_test_creep_speed", 0));
//		}
//		else
//		{
//			Robot.drive.setVelocityZero();
//		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	double should_finish = SmartDashboard.getNumber("vision_test_end_flag", 0);
    	//System.out.println("should_finish: " + totalTimer.get());
        return  totalTimer.get() > 20.0;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("end()!!!!!!!!");
		if(pw!= null)
		{
			StringBuilder sb = new StringBuilder();
			sb.append("Time, Dist, Area\n");
			for (int i = 0; i < debug_list_time.size(); i++) {
				sb.append(debug_list_time.get(i) + ", " + debug_list_dist.get(i) + ", " + debug_list_area.get(i) +  "\n");
			}
	        
		    pw.write(sb.toString());	
			pw.close();
			pw = null;
		}
    	System.out.println("VisionDistanceCalibrate end!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
