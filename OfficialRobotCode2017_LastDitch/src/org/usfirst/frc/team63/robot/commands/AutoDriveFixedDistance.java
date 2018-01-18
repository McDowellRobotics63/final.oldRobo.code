package org.usfirst.frc.team63.robot.commands;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team63.robot.Kinematics;
import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.RobotMap;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;
import org.usfirst.frc.team63.robot.util.SynchronousPID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class AutoDriveFixedDistance extends Command {
	
	private RobotDriveDirection desired_direction = RobotDriveDirection.ROBOT_FORWARD;
	private double desired_distance = 0;
	private double distance_traveled = 0;	
		
    private SynchronousPID sync_pid;
    private double last_pid_speed = 0;
	
	private double last_front_left_inches = 0;
	private double last_rear_left_inches = 0;
	private double last_front_right_inches = 0;
	private double last_rear_right_inches = 0;
	
	private final boolean DEBUG_FLAG = true;
	List<Double> debug_list_time = null;
	List<Double> debug_list_ref = null;
	List<Double> debug_list_dist = null;
	List<Double> debug_list_pid_spd = null;
	List<Double> debug_list_actual_spd = null;
	List<Integer> debug_list_on_target = null;
	
	private PrintWriter pw = null;
	
	private final Timer onTargetTimer;
	private final Timer totalTimer;
	
	private static final double dt = 0.02;
	
    public AutoDriveFixedDistance(RobotDriveDirection direction, double distance_inches) {
    	requires(Robot.drive);
    	setInterruptible(false);
    	
    	System.out.println("AutoDriveFixedDistance::AutoDriveFixedDistance()");
    	
    	desired_direction = direction;
    	desired_distance = distance_inches;
    	
    	if(desired_direction == RobotDriveDirection.ROBOT_BACK || 
 	    	   desired_direction == RobotDriveDirection.ROBOT_RIGHT ||
 	    	   desired_direction == RobotDriveDirection.GEAR_FORWARD ||
 	    	   desired_direction == RobotDriveDirection.GEAR_RIGHT)
	 	{
	 		desired_distance = -desired_distance;
	 	}
    	
    	onTargetTimer = new Timer();
    	totalTimer = new Timer();
    	
    	sync_pid = new SynchronousPID(RobotMap.kP_position, RobotMap.kI_position, RobotMap.kD_position);
    	sync_pid.setInputRange(-RobotMap.max_distance_position, RobotMap.max_distance_position);
    	sync_pid.setOutputRange(-RobotMap.max_speed_position, RobotMap.max_speed_position);
    	sync_pid.setContinuous(false);
    	
    	System.out.println("END OF AutoDriveFixedDistance::AutoDriveFixedDistance()!!");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("AutoDriveFixedDistance::initialize()");
    	
    	if(DEBUG_FLAG)
    	{
			try {
				pw = new PrintWriter(new File("/home/lvuser/pid_position_" + System.currentTimeMillis() + ".csv"));
				debug_list_time = new ArrayList<>();
				debug_list_ref = new ArrayList<>();
				debug_list_dist = new ArrayList<>();
				debug_list_pid_spd = new ArrayList<>();
				debug_list_actual_spd = new ArrayList<>();
				debug_list_on_target = new ArrayList<>();
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
    	}
    
    	//Start counting distance from zero
    	Robot.drive.resetEncoders();
    	//Maintain the current heading while driving in the desired robot direction
    	Robot.drive.mDriveHelper.ResetOrientationSetpoint();
    	//Set the desired distance to travel for the PID    	
    	sync_pid.setSetpoint(desired_distance);        
        
    	//Start the on target timer.. it will be restarted whenever we are NOT on target
    	onTargetTimer.reset();
    	onTargetTimer.start();
    	
    	totalTimer.reset();
    	totalTimer.start();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {   
    	System.out.println("AutoDriveFixedDistance::execute()");
    	
    	double curr_front_left_inches = Robot.drive.getFrontLeftDistanceInches();
    	double curr_rear_left_inches = Robot.drive.getRearLeftDistanceInches();
    	//double curr_front_right_inches = Robot.drive.getFrontRightDistanceInches();
    	double curr_front_right_inches = 0;
    	double curr_rear_right_inches = Robot.drive.getRearRightDistanceInches();	
    	
    	double pid_speed = 0;
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
    	
    	if(desired_direction == RobotDriveDirection.ROBOT_FORWARD ||
    	   desired_direction == RobotDriveDirection.ROBOT_BACK ||
    	   desired_direction == RobotDriveDirection.GEAR_LEFT ||
    	   desired_direction == RobotDriveDirection.GEAR_RIGHT)
    	{
    		delta_dist = delta.dx;
    	}
    	else
    	{
    		delta_dist = delta.dy;
    	}    		
    	
    	distance_traveled = distance_traveled + delta_dist;
    	
    	System.out.println("desired_distance:" + desired_distance + ", distance_traveled:" + distance_traveled);
    	if(Math.abs(desired_distance - distance_traveled) > RobotMap.kTolerance_position)
    	{
    		onTargetTimer.reset();
        	onTargetTimer.start();
        	
	    	pid_speed = sync_pid.calculate(distance_traveled);	    		    	
	    	
	    	// Maintain minimum speed command
	    	if(Math.abs(pid_speed) < RobotMap.creep_speed_position)
	    	{
	    		pid_speed = Math.signum(pid_speed) * RobotMap.creep_speed_position;
	    	}
	    	else
	    	{
		    	if(pid_speed > last_pid_speed)
		    	{
		    		pid_speed = Math.min(pid_speed, last_pid_speed + RobotMap.ramp_rate_position * 0.02);
		    	}
		    	else
		    	{
		    		pid_speed = Math.max(pid_speed, last_pid_speed - RobotMap.ramp_rate_position * 0.02);
		    	}
	    	}
	    	
	        // Maintain maximum speed command
	    	if(Math.abs(pid_speed) > RobotMap.max_speed_position)
	    	{
	    		pid_speed = Math.signum(pid_speed) * RobotMap.max_speed_position;
	    	}
	    		
	    	last_pid_speed = pid_speed;
	    	
	    	System.out.println("<AutoDriveFixedDistance> Remaining Distance: " + (desired_distance - distance_traveled) + ", Speed: " + pid_speed);
	    	
	        switch (desired_direction) {
	        case ROBOT_FORWARD: 
	        case GEAR_LEFT:
	        case GEAR_RIGHT:
	        case ROBOT_BACK:
	        	Robot.drive.operatorControl(-pid_speed, 0, 0);
	            break;
	        case ROBOT_LEFT:
	        case ROBOT_RIGHT:
	        case GEAR_FORWARD:
	        case GEAR_BACK:   
	        	
	        	Robot.drive.operatorControl(pid_speed, 0, 0);
	            break;
	        default:
	            System.out.println("huh??");
	            break;
	        }
    	}
    	else
    	{
    		pid_speed = 0;
    		Robot.drive.setVelocityZero();
    		System.out.println("<AutoDriveFixedDistance> Within tolerance, waiting for on target time..");
    	}
    	
    	if(DEBUG_FLAG)
    	{
    		debug_list_time.add(totalTimer.get());
    		debug_list_ref.add(desired_distance);
    		debug_list_dist.add(distance_traveled);
    		debug_list_pid_spd.add(pid_speed);
    		debug_list_actual_spd.add(delta_dist/dt);	// dx/dt
    		debug_list_on_target.add((Math.abs(desired_distance - distance_traveled) > RobotMap.kTolerance_position) ? 0: 1);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return onTargetTimer.get() > RobotMap.kOnTargetTime_position;
    	//return totalTimer.get() > 15;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setVelocityZero();
		if(pw!= null)
		{
			StringBuilder sb = new StringBuilder();
			sb.append("Time, Ref, Dist, Cmd, Spd, OnTarget\n");
			for (int i = 0; i < debug_list_ref.size(); i++) {
				sb.append(debug_list_time.get(i) + ", " + debug_list_ref.get(i) + ", " + debug_list_dist.get(i) + ", " + debug_list_pid_spd.get(i) + ", " + debug_list_actual_spd.get(i) + ", " + debug_list_on_target.get(i) +  "\n");
			}
	        
		    pw.write(sb.toString());	
			pw.close();
			pw = null;
		}
    	System.out.println("AutoDriveFixedDistance end!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
