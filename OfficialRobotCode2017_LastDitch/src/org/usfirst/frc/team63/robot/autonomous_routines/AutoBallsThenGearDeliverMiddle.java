package org.usfirst.frc.team63.robot.autonomous_routines;

import org.usfirst.frc.team63.robot.Robot;
import org.usfirst.frc.team63.robot.commands.AutoDriveFixedDistance;
import org.usfirst.frc.team63.robot.commands.AutoRotateFixedDegrees;
import org.usfirst.frc.team63.robot.commands.AutoVisionAdjustLeftRight;
import org.usfirst.frc.team63.robot.commands.AutoVisionApproachTarget;
import org.usfirst.frc.team63.robot.commands.GearDeliveryCommandGroup;
import org.usfirst.frc.team63.robot.commands.WaitForVisionDataUpdate;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorCloseCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorOpenCommand;
import org.usfirst.frc.team63.robot.simple_commands.DelayCommand;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotRotateDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoBallsThenGearDeliverMiddle extends CommandGroup {
    
    public AutoBallsThenGearDeliverMiddle() {
    	
    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 36.0));
        addSequential(new AutoRotateFixedDegrees(RobotRotateDirection.ROBOT_CCW, 45));
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_BACK, 36.0));
        addSequential(new BallDoorOpenCommand());
        addSequential(new DelayCommand(3.0, Robot.balls));
        addSequential(new BallDoorCloseCommand());
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 36.0));
        addSequential(new AutoRotateFixedDegrees(RobotRotateDirection.ROBOT_CCW, 45));
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_FORWARD, 120.0));
        addSequential(new AutoDriveFixedDistance(RobotDriveDirection.GEAR_FORWARD, 24.0));
        
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data        
        addSequential(new AutoVisionApproachTarget(12.0)); //get within 12 inches of the target
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data        
        addSequential(new AutoVisionAdjustLeftRight()); //make sure we are centered on the target
        addSequential(new WaitForVisionDataUpdate()); //Wait for valid updated vision data        
        addSequential(new AutoVisionApproachTarget(0.0)); //close the final gap
    	addSequential(new GearDeliveryCommandGroup()); //deliver the gear
    	addSequential(new AutoDriveFixedDistance(RobotDriveDirection.ROBOT_BACK, 18.0)); //back off the peg
    }
}
