package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearMechanismSubsystem extends Subsystem {
	
	Solenoid gearAdjustExtend;
	Solenoid gearAdjustRetract;
	Solenoid gearDoorDrop;
	Solenoid gearDoorRaise;
	Solenoid gearPusherExtend;
	Solenoid gearPusherRetract;
	
	
	
	public GearMechanismSubsystem()
	{
		
		gearAdjustExtend = new Solenoid(6, RobotMap.GEAR_ADJUST_EXTEND_SOLENOID);
		gearAdjustRetract = new Solenoid(6, RobotMap.GEAR_ADJUST_RETRACT_SOLENOID);
		gearDoorDrop = new Solenoid(6, RobotMap.GEAR_DOOR_DROP_SOLENOID);
		gearDoorRaise = new Solenoid(6, RobotMap.GEAR_DOOR_RAISE_SOLENOID);
		gearPusherExtend = new Solenoid(6, RobotMap.GEAR_PUSHER_EXTEND_SOLENOID);
		gearPusherRetract = new Solenoid(6, RobotMap.GEAR_PUSHER_RETRACT_SOLENOID);
		
		gearAdjustExtend.set(true);
		gearAdjustRetract.set(false);
		gearDoorDrop.set(false);
		gearDoorRaise.set(true);
		gearPusherExtend.set(false);
		gearPusherRetract.set(true);
		
	}
	
	public void gearAdjustExtend()
	{
		gearAdjustExtend.set(true);
		gearAdjustRetract.set(false);
	}
	
	public void gearAdjustRetract()
	{
		gearAdjustExtend.set(false);
		gearAdjustRetract.set(true);
	}
	
	public void gearDoorDrop()
	{
		gearDoorDrop.set(true);
		gearDoorRaise.set(false);
	}
	
	public void gearDoorRaise()
	{
		gearDoorDrop.set(false);
		gearDoorRaise.set(true);
	}
	
	public void gearPusherExtend()
	{
		gearPusherExtend.set(true);
		gearPusherRetract.set(false);
	}
	
	public void gearPusherRetract()
	{
		gearPusherExtend.set(false);
		gearPusherRetract.set(true);
	}
	
	
	

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
