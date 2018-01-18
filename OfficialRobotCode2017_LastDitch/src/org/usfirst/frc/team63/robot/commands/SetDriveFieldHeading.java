package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetDriveFieldHeading extends Command {

    public enum FieldOrientationOverride {
        NONE, TWELVE_OCLOCK, THREE_OCLOCK, SIX_OCLOCK, NINE_OCLOCK, GEAR_RETRIEVE, GEAR_DELIVER_LEFT, GEAR_DELIVER_RIGHT, GEAR_DELIVER_MIDDLE;
    }
    
    private FieldOrientationOverride heading_override = FieldOrientationOverride.NONE;
    private double setpoint = 0;
    
    public SetDriveFieldHeading(FieldOrientationOverride value) {
    	requires(Robot.drive);    	
    	setInterruptible(false);
    	heading_override = value;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        switch (heading_override) {
        case NONE:        	
            break;
        case TWELVE_OCLOCK:
        	setpoint = 0;
            break;
        case THREE_OCLOCK:
        	setpoint = 90;
            break;
        case SIX_OCLOCK:
        	setpoint = 179.8;
            break;
        case NINE_OCLOCK:
        	setpoint = -90;
            break;
            
        case GEAR_RETRIEVE:
        	setpoint = 45;
        	break;	        	
        case GEAR_DELIVER_LEFT:
        	setpoint = 45;
        	break;	        	
        case GEAR_DELIVER_RIGHT:
        	setpoint = 135;
        	break;	        	
        case GEAR_DELIVER_MIDDLE:
        	setpoint = 95;
        	break;
        	
        default:
            System.out.println("Unexpected field orientation value: " + heading_override);
            break;
        }
        
        Robot.drive.mDriveHelper.SetOrientationSetpoint(setpoint);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
