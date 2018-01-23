package org.usfirst.frc.team63.robot.autonomous_routines;

import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistance;
import org.usfirst.frc.team63.robot.commands.AutoRotateFixedDegrees;
import org.usfirst.frc.team63.robot.commands.AutoVisionAdjustLeftRight;
import org.usfirst.frc.team63.robot.commands.AutoVisionApproachTarget;
import org.usfirst.frc.team63.robot.commands.GearDeliveryCommandGroup;
import org.usfirst.frc.team63.robot.commands.WaitForVisionDataUpdate;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotRotateDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoGearDeliver extends CommandGroup {
    public enum GearDeliverPosition {
        LEFT_PEG, MIDDLE_PEG, RIGHT_PEG;
    }
    
    public AutoGearDeliver(GearDeliverPosition target) {
    	
    	if(target == GearDeliverPosition.LEFT_PEG)
    	{
        	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 60.0));
            addSequential(new AutoRotateFixedDegrees(RobotRotateDirection.ROBOT_CCW, 45));
    	}
    	else if(target == GearDeliverPosition.MIDDLE_PEG)
    	{
        	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 60.0));
            addSequential(new AutoRotateFixedDegrees(RobotRotateDirection.ROBOT_CCW, 90));
    	}
    	else if(target == GearDeliverPosition.RIGHT_PEG)
    	{
        	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 60.0));
            addSequential(new AutoRotateFixedDegrees(RobotRotateDirection.ROBOT_CCW, 135));
    	}
        
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        addSequential(new AutoVisionApproachTarget(12.0)); //get within 12 inches of the target
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        addSequential(new AutoVisionAdjustLeftRight()); //make sure we are centered on the target
//        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data      
//        addSequential(new AutoVisionApproachTarget(0.0)); //close the final gap
//    	addSequential(new GearDeliveryCommandGroup()); //deliver the gear
//    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.GEAR_BACK, 18.0)); //back off the peg
    }
}
