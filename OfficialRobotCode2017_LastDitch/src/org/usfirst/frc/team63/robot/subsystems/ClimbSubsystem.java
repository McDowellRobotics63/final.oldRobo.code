package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ClimbSubsystem extends Subsystem {


	TalonSRX RobotWinchMotor;
	
	public ClimbSubsystem()
	{		
		RobotWinchMotor = new TalonSRX(RobotMap.ROBOT_CLIMB_MOTOR);			
	}
	
	public void RobotClimb()
	{
		 this.RobotWinchMotor.set(ControlMode.PercentOutput, -1.0);
		 SmartDashboard.putNumber("climb_current", RobotWinchMotor.getOutputCurrent());
	}
	
	public void RobotStopClimb()
	{
		this.RobotWinchMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void RobotDownClimb()
	{
		 this.RobotWinchMotor.set(ControlMode.PercentOutput, 1.0);
		 SmartDashboard.putNumber("climb_current", RobotWinchMotor.getOutputCurrent());
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
