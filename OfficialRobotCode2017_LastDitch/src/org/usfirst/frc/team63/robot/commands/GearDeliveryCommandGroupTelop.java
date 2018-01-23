package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.simple_commands.DelayCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearAdjustExtendCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearDoorDropCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearDoorRaiseCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearPusherExtendCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearPusherRetractCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GearDeliveryCommandGroupTelop extends CommandGroup {

    public GearDeliveryCommandGroupTelop() {
    	addSequential(new GearDoorDropCommand());
    	addSequential(new DelayCommand(0.25, Robot.gears));
    	addSequential(new GearPusherExtendCommand());
    	addSequential(new DelayCommand(0.5, Robot.gears));
    	addSequential(new GearPusherRetractCommand());
    	addSequential(new GearAdjustExtendCommand());
    	addSequential(new DelayCommand(5.0, Robot.gears));
    	addSequential(new GearDoorRaiseCommand());
    }
}
