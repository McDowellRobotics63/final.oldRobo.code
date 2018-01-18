package org.usfirst.frc.team63.robot.simple_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DelayCommand extends Command {
	
	public final Timer delayTimer;
	double delayAsked;

    public DelayCommand(double delay, Subsystem sub) {
    	requires(sub);
    	delayTimer = new Timer();
    	delayAsked = delay;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	delayTimer.reset();
    	delayTimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return delayTimer.get() > delayAsked;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
