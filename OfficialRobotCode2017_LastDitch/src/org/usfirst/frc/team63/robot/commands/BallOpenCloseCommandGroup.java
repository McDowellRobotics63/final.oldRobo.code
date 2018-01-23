package org.usfirst.frc.team63.robot.commands;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorCloseCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorOpenCommand;
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
public class BallOpenCloseCommandGroup extends CommandGroup {

    public BallOpenCloseCommandGroup() {
//        addSequential(new BallDoorOpenCommand());
//        addSequential(new DelayCommand(2.0, Robot.balls));
//        addSequential(new BallDoorCloseCommand());
//        addSequential(new DelayCommand(0.25, Robot.balls));
//        addSequential(new BallDoorOpenCommand());
//        addSequential(new DelayCommand(1.5, Robot.balls));
//        addSequential(new BallDoorCloseCommand());
//        addSequential(new DelayCommand(0.25, Robot.balls));
//        addSequential(new BallDoorOpenCommand());
    	
      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));
      
      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

      addSequential(new BallDoorOpenCommand());
      addSequential(new DelayCommand(1.0, Robot.balls));
      addSequential(new BallDoorCloseCommand());
      addSequential(new DelayCommand(0.25, Robot.balls));

    }
}
