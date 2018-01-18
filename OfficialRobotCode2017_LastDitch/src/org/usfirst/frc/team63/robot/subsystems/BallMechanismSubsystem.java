package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BallMechanismSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	Victor RobotBallMotor;
	Spark Tickler;
	Solenoid ballDoorOpen;
	Solenoid ballDoorClose;
	
	public BallMechanismSubsystem()
	{		
		RobotBallMotor = new Victor(RobotMap.BALL_GRAB_MOTOR);
		Tickler = new Spark(RobotMap.BALL_TICKLER_MOTOR);
		ballDoorOpen = new Solenoid(6, RobotMap.BALL_DOOR_OPEN_SOLENOID);
		ballDoorClose = new Solenoid(6, RobotMap.BALL_DOOR_CLOSE_SOLENOID);		
		
		ballDoorOpen.set(false);
		ballDoorClose.set(true);		
	}
	
	public void ballDoorOpen()
	{
		ballDoorOpen.set(true);
		ballDoorClose.set(false);	
		//this.Tickler.set(SmartDashboard.getNumber("TicklerVal", 0.5));
		this.Tickler.set(-0.5);
	}
	
	public void ballDoorClose()
	{
		ballDoorOpen.set(false);
		ballDoorClose.set(true);
		this.Tickler.set(0.0);
	}
	
	public void BallLift()
	{
		this.RobotBallMotor.set(SmartDashboard.getNumber("BallLiftVal", 0));
	}

	public void BallStop()
	{
		this.RobotBallMotor.set(0.0);
	}
	
	public void BallDejam()
	{
		this.RobotBallMotor.set(0.75);
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		//setDefaultCommand(new BallGrabCommand());
	}
}
