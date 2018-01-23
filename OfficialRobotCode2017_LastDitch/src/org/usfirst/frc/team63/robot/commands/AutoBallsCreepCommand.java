package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.RobotMap;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoBallsCreepCommand extends Command {
    
    private RobotDriveDirection dir = RobotDriveDirection.GEAR_FORWARD;
    
    public AutoBallsCreepCommand(RobotDriveDirection value) {
    	requires(Robot.drive);    	
    	setInterruptible(false);
    	dir = value;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        switch (dir) {
        case GEAR_FORWARD:
        	Robot.drive.moveStraightLeftRight(-RobotMap.kCreepGearForwardSpeed);        	
            break;
        case GEAR_BACK:   
        	Robot.drive.moveStraightLeftRight(RobotMap.kCreepGearBackSpeed);
            break;
        case GEAR_LEFT:
        	Robot.drive.moveStraightForwardBack(RobotMap.kCreepGearLeftSpeed);        	
            break;
        case GEAR_RIGHT:
        	Robot.drive.moveStraightForwardBack(-RobotMap.kCreepGearRightSpeed);        	
            break;
        	
        default:
            System.out.println("Unexpected creep value: " + dir);
            break;
        }        
       
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
