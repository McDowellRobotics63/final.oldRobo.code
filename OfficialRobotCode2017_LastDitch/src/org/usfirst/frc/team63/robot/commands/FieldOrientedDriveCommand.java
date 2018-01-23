package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class FieldOrientedDriveCommand extends Command {
		
    public FieldOrientedDriveCommand() {
    	requires(Robot.drive);
    	setInterruptible(true);    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double fwd_back = Robot.oi.stick1.getRawAxis(RobotMap.XBOX_LEFT_Y_AXIS);
    	double left_right = Robot.oi.stick1.getRawAxis(RobotMap.XBOX_LEFT_X_AXIS);
    	double rotate = Robot.oi.stick1.getRawAxis(RobotMap.XBOX_RIGHT_X_AXIS);
             
    	Robot.drive.operatorControl(fwd_back, left_right, rotate);    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
