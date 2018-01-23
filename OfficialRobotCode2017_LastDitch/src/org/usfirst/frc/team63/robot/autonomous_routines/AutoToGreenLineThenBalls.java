package org.usfirst.frc.team63.robot.autonomous_routines;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.commands.AutoBallsCreepCommand;
import org.usfirst.frc.team63.robot.commands.AutoDriveBackForTheBalls;
import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistance;
import org.usfirst.frc.team63.robot.commands.AutoRotateFixedDegrees;
import org.usfirst.frc.team63.robot.commands.AutoRotateToHeading;
import org.usfirst.frc.team63.robot.commands.BallOpenCloseCommandGroup;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorCloseCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorOpenCommand;
import org.usfirst.frc.team63.robot.simple_commands.DelayCommand;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotRotateDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoToGreenLineThenBalls extends CommandGroup {
	
    public enum AllianceColor {
        RED_ALLIANCE, BLUE_ALLIANCE;
    }
    public AutoToGreenLineThenBalls(AllianceColor Alliance) {
    	
    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 120.0));
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_BACK, 60.0));
    	if (Alliance == AllianceColor.RED_ALLIANCE)
        {
    		addSequential(new AutoRotateToHeading(60));
        }
    	else
    	{
    		addSequential(new AutoRotateToHeading(-60));
    	}
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_BACK, 59));
        addParallel(new BallOpenCloseCommandGroup());
        addSequential(new AutoDriveBackForTheBalls());        
        
    }
}
