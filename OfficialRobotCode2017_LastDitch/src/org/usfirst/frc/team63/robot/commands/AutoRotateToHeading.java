package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoRotateToHeading extends Command {
	
	private static final double kTolerance = 20;
	private static final double kOnTargetTime = 0.25;
	private double desired_heading = 0;
	public final Timer onTargetTimer;
	
    public AutoRotateToHeading(double val) {
    	requires(Robot.drive);
    	setInterruptible(true);
    	desired_heading = val;
    	onTargetTimer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.mDriveHelper.SetOrientationSetpoint(desired_heading);
    	onTargetTimer.reset();
    	onTargetTimer.start();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drive.operatorControl(0, 0, 0);
    	System.out.println("desired_heading: " + desired_heading + ", Robot.drive.getGyroAngleDegrees(): " + Robot.drive.getGyroAngleDegrees());
    	if(Math.abs(desired_heading - Robot.drive.getGyroAngleDegrees()) > kTolerance)
    	{
    		onTargetTimer.reset();
        	onTargetTimer.start();
    	}    		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return onTargetTimer.get() > kOnTargetTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Auto rotate end!!");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
