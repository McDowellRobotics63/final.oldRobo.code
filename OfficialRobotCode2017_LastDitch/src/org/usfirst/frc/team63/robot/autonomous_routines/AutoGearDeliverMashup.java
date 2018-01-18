package org.usfirst.frc.team63.robot.autonomous_routines;

import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistance;
import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistanceStrafe;
import org.usfirst.frc.team63.robot.commands.AutoRotateFixedDegrees;
import org.usfirst.frc.team63.robot.commands.AutoRotateToHeading;
import org.usfirst.frc.team63.robot.commands.AutoVisionAdjustLeftRight;
import org.usfirst.frc.team63.robot.commands.AutoVisionApproachTarget;
import org.usfirst.frc.team63.robot.commands.GearDeliveryCommandGroup;
import org.usfirst.frc.team63.robot.commands.WaitForVisionDataUpdate;
import org.usfirst.frc.team63.robot.simple_commands.GearDoorRaiseCommand;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotRotateDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoGearDeliverMashup extends CommandGroup {
    
    public AutoGearDeliverMashup() {
    	
    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 61.0));
        addSequential(new AutoRotateToHeading(103));
        addSequential(new AutoDriveFixedDistanceStrafe(RobotDriveDirection.GEAR_FORWARD, 55.0));
        
    	addSequential(new GearDeliveryCommandGroup()); //deliver the gear
    	addSequential(new AutoDriveFixedDistanceStrafe(RobotDriveDirection.GEAR_BACK, 18.0)); //back off the peg
    	addSequential(new GearDoorRaiseCommand());
    }
}
