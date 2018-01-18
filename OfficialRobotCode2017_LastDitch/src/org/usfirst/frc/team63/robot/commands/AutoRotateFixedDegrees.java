package org.usfirst.frc.team63.robot.commands;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.RobotMap;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotRotateDirection;
import org.usfirst.frc.team63.robot.util.SynchronousPID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class AutoRotateFixedDegrees extends Command {
	
	private RobotRotateDirection desired_direction = RobotRotateDirection.ROBOT_CCW;
	private double desired_degrees = 0;
	private double last_degrees = 0;
	private double degrees_traveled = 0;	
		
    private SynchronousPID sync_pid;
	
	private final boolean DEBUG_FLAG = true;
	List<Double> debug_list_time = null;
	List<Double> debug_list_ref = null;
	List<Double> debug_list_degrees = null;
	List<Double> debug_list_pid_spd = null;
	List<Double> debug_list_actual_spd = null;
	List<Integer> debug_list_on_target = null;
	
	private PrintWriter pw = null;
	
	private final Timer onTargetTimer;
	private final Timer totalTimer;
	
	private static final double dt = 0.02;
	
    public AutoRotateFixedDegrees(RobotRotateDirection direction, double degrees) {
    	requires(Robot.drive);
    	setInterruptible(false);
    	
    	desired_direction = direction;
    	desired_degrees = degrees;
    	
    	if(desired_direction == RobotRotateDirection.ROBOT_CW)
	 	{
    		desired_degrees = -desired_degrees;
	 	}
    	
    	onTargetTimer = new Timer();
    	totalTimer = new Timer();
    	
    	sync_pid = new SynchronousPID(RobotMap.kP_rotate, RobotMap.kI_rotate, RobotMap.kD_rotate);
    	sync_pid.setInputRange(-RobotMap.max_degrees_rotate, RobotMap.max_degrees_rotate);
    	sync_pid.setOutputRange(-RobotMap.max_speed_rotate, RobotMap.max_speed_rotate);
    	sync_pid.setContinuous(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	if(DEBUG_FLAG)
    	{
			try {
				pw = new PrintWriter(new File("pid_rotate_" + System.currentTimeMillis() + ".csv"));
				debug_list_time = new ArrayList<>();
				debug_list_ref = new ArrayList<>();
				debug_list_degrees = new ArrayList<>();
				debug_list_pid_spd = new ArrayList<>();
				debug_list_actual_spd = new ArrayList<>();
				debug_list_on_target = new ArrayList<>();
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
    	}
    	
    	last_degrees = Robot.drive.getGyroAngleDegrees();
    
    	//Set the desired distance to travel for the PID    	
    	sync_pid.setSetpoint(desired_degrees);        
        
    	//Start the on target timer.. it will be restarted whenever we are NOT on target
    	onTargetTimer.reset();
    	onTargetTimer.start();
    	
    	totalTimer.reset();
    	totalTimer.start();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double curr_degrees = Robot.drive.getGyroAngleDegrees();
    	
    	double pid_speed = 0;
    	double delta_degrees = 0;
    	
    	
    	delta_degrees = curr_degrees - last_degrees;    	
    	last_degrees = curr_degrees;
    	
    	degrees_traveled = degrees_traveled + delta_degrees;
    	
    	if(Math.abs(desired_degrees - degrees_traveled) > RobotMap.kTolerance_rotate)
    	{
    		onTargetTimer.reset();
        	onTargetTimer.start();
        	
	    	pid_speed = sync_pid.calculate(degrees_traveled);
	    	
	    	// Maintain minimum speed command
	    	if(Math.abs(pid_speed) < RobotMap.creep_speed_rotate)
	    	{
	    		pid_speed = Math.signum(pid_speed) * RobotMap.creep_speed_rotate;
	    	}
	    	
	        // Maintain maximum speed command
	    	if(Math.abs(pid_speed) > RobotMap.max_speed_rotate)
	    	{
	    		pid_speed = Math.signum(pid_speed) * RobotMap.max_speed_rotate;
	    	}
	    		
	    	System.out.println("<AutoRotateFixedDegrees> Remaining Distance: " + (desired_degrees - degrees_traveled) + ", Speed: " + pid_speed);
	    	
	        switch (desired_direction) {
	        case ROBOT_CCW:   
	        case ROBOT_CW:       
	        	Robot.drive.rotateDrive(pid_speed);
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
    		System.out.println("<AutoRotateFixedDegrees> Within tolerance, waiting for on target time..");
    	}
    	
    	if(DEBUG_FLAG)
    	{
    		debug_list_time.add(totalTimer.get());
    		debug_list_ref.add(desired_degrees);
    		debug_list_degrees.add(degrees_traveled);
    		debug_list_pid_spd.add(pid_speed);
    		debug_list_actual_spd.add(delta_degrees/dt);	// dtheta/dt
    		debug_list_on_target.add((Math.abs(desired_degrees - degrees_traveled) > RobotMap.kTolerance_rotate) ? 0: 1);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return onTargetTimer.get() > RobotMap.kOnTargetTime_rotate;
    }

    // Called once after isFinished returns true
    protected void end() {
		if(pw!= null)
		{
			StringBuilder sb = new StringBuilder();
			sb.append("Time, Ref, Dist, Cmd, Spd, OnTarget\n");
			for (int i = 0; i < debug_list_ref.size(); i++) {
				sb.append(debug_list_time.get(i) + ", " + debug_list_ref.get(i) + ", " + debug_list_degrees.get(i) + ", " + debug_list_pid_spd.get(i) + ", " + debug_list_actual_spd.get(i) + ", " + debug_list_on_target.get(i) +  "\n");
			}
	        
		    pw.write(sb.toString());	
			pw.close();
			pw = null;
		}
    	System.out.println("AutoRotateFixedDegrees end!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
