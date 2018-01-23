package org.usfirst.frc.team63.robot.autonomous_routines;

import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistance;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoDriveToGreenLine extends CommandGroup {
	
    public AutoDriveToGreenLine() {   
    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 120.0));
    }
}
