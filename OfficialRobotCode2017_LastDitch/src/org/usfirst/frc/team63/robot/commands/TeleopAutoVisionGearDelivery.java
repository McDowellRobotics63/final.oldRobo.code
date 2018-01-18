package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.simple_commands.DriveStopCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TeleopAutoVisionGearDelivery extends CommandGroup {

    public TeleopAutoVisionGearDelivery() {
    	addSequential(new DriveStopCommand());
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        
        addSequential(new AutoVisionApproachTarget(48.0)); //get within 48 inches of the target
        
        addSequential(new DriveStopCommand());
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        
        addSequential(new AutoVisionAdjustLeftRight()); //make sure we are centered on the target
        
        addSequential(new DriveStopCommand());
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        
        addSequential(new AutoVisionApproachTarget(12.0)); //get within 12 inches of the target
        
        addSequential(new DriveStopCommand());
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        
        addSequential(new AutoVisionAdjustLeftRight()); //make sure we are centered on the target
        
        addSequential(new DriveStopCommand());
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data
        
        addSequential(new AutoVisionApproachTarget(0.0)); //close the final gap        
        addSequential(new DriveStopCommand()); //make sure we aren't moving
    	addSequential(new GearDeliveryCommandGroup()); //deliver the gear    	      	
     }
}
