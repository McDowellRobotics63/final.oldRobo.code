package org.usfirst.frc.team63.robot;

import org.usfirst.frc.team63.robot.XboxDPadButton.DPAD_BUTTON;
import org.usfirst.frc.team63.robot.commands.GearDeliveryCommandGroup;
import org.usfirst.frc.team63.robot.commands.GearDeliveryCommandGroupTelop;
import org.usfirst.frc.team63.robot.commands.ResetDriveHeadingReference;
import org.usfirst.frc.team63.robot.commands.SetDriveCreepCommand;
import org.usfirst.frc.team63.robot.commands.SetDriveFieldHeading.FieldOrientationOverride;
import org.usfirst.frc.team63.robot.commands.VisionDistanceCalibrate;
import org.usfirst.frc.team63.robot.simple_commands.BallDejamCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorCloseCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallDoorOpenCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallGrabCommand;
import org.usfirst.frc.team63.robot.simple_commands.BallStopCommand;
import org.usfirst.frc.team63.robot.simple_commands.DriveStopCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearAdjustExtendCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearAdjustRetractCommand;
import org.usfirst.frc.team63.robot.simple_commands.RobotClimbCommand;
import org.usfirst.frc.team63.robot.simple_commands.RobotStopClimbCommand;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem.RobotDriveDirection;
import org.usfirst.frc.team63.robot.commands.SetDriveFieldHeading;
import org.usfirst.frc.team63.robot.commands.TeleopAutoVisionGearDelivery;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick stick1 = new Joystick(0);
	public Joystick stick2 = new Joystick(1);
	
	 //CONTROLLER 1 BUTTONS	 
	 public Button stick1_X = new JoystickButton(stick1 , RobotMap.XBOX_X);
	 public Button stick1_Y = new JoystickButton(stick1 , RobotMap.XBOX_Y);
	 public Button stick1_A = new JoystickButton(stick1 , RobotMap.XBOX_A);
	 public Button stick1_B = new JoystickButton(stick1 , RobotMap.XBOX_B);
	 public Button stick1_LB = new JoystickButton(stick1 , RobotMap.XBOX_LEFT_BUMPER);
	 public Button stick1_RB = new JoystickButton(stick1 , RobotMap.XBOX_RIGHT_BUMPER);
	 public Button stick1_start = new JoystickButton(stick1 , RobotMap.XBOX_START);
	 public Button stick1_back = new JoystickButton(stick1 , RobotMap.XBOX_BACK);
	 public Button stick1_dpadUp = new XboxDPadButton(stick1, DPAD_BUTTON.DPAD_UP);
	 public Button stick1_dpadRight = new XboxDPadButton(stick1, DPAD_BUTTON.DPAD_RIGHT);
	 public Button stick1_dpadDown = new XboxDPadButton(stick1, DPAD_BUTTON.DPAD_DOWN);
	 public Button stick1_dpadLeft = new XboxDPadButton(stick1, DPAD_BUTTON.DPAD_LEFT);
	 
	 
	 //CONTROLLER 2 BUTTONS
	 public Button stick2_X = new JoystickButton(stick2 , RobotMap.XBOX_X);
	 public Button stick2_Y = new JoystickButton(stick2 , RobotMap.XBOX_Y);
	 public Button stick2_A = new JoystickButton(stick2 , RobotMap.XBOX_A);
	 public Button stick2_B = new JoystickButton(stick2 , RobotMap.XBOX_B);
	 public Button stick2_LB = new JoystickButton(stick2 , RobotMap.XBOX_LEFT_BUMPER);
	 public Button stick2_RB = new JoystickButton(stick2 , RobotMap.XBOX_RIGHT_BUMPER);
	 public Button stick2_start = new JoystickButton(stick2 , RobotMap.XBOX_START);
	 public Button stick2_back = new JoystickButton(stick2 , RobotMap.XBOX_BACK);
	 public Button stick2_dpadUp = new XboxDPadButton(stick2, DPAD_BUTTON.DPAD_UP);
	 public Button stick2_dpadRight = new XboxDPadButton(stick2, DPAD_BUTTON.DPAD_RIGHT);
	 public Button stick2_dpadDown = new XboxDPadButton(stick2, DPAD_BUTTON.DPAD_DOWN);
	 public Button stick2_dpadLeft = new XboxDPadButton(stick2, DPAD_BUTTON.DPAD_LEFT);	 	 
	 
	
	public OI()
	{
		//Joystick 1
		stick1_back.whenPressed(new ResetDriveHeadingReference());
		stick1_Y.whenPressed(new SetDriveFieldHeading(FieldOrientationOverride.GEAR_RETRIEVE));
		stick1_X.whenPressed(new SetDriveFieldHeading(FieldOrientationOverride.GEAR_DELIVER_LEFT));
		stick1_B.whenPressed(new SetDriveFieldHeading(FieldOrientationOverride.GEAR_DELIVER_RIGHT));
		stick1_A.whenPressed(new SetDriveFieldHeading(FieldOrientationOverride.GEAR_DELIVER_MIDDLE));
		stick1_dpadUp.whileHeld(new SetDriveCreepCommand(RobotDriveDirection.GEAR_LEFT));
		stick1_dpadUp.whenReleased(new DriveStopCommand());
		stick1_dpadDown.whileHeld(new SetDriveCreepCommand(RobotDriveDirection.GEAR_RIGHT));
		stick1_dpadDown.whenReleased(new DriveStopCommand());
		stick1_dpadLeft.whileHeld(new SetDriveCreepCommand(RobotDriveDirection.GEAR_BACK));
		stick1_dpadLeft.whenReleased(new DriveStopCommand());
		stick1_dpadRight.whileHeld(new SetDriveCreepCommand(RobotDriveDirection.GEAR_FORWARD));
		stick1_dpadRight.whenReleased(new DriveStopCommand());
		
		//Joystick 2
		stick2_dpadUp.whileHeld(new RobotClimbCommand());
		stick2_dpadUp.whenReleased(new RobotStopClimbCommand());
//		stick2_dpadDown.whileHeld(new RobotDownClimbCommand());
//		stick2_dpadDown.whenReleased(new RobotStopClimbCommand());
		//stick2_Y.whileHeld(new TeleopAutoVisionGearDelivery());
		stick2_Y.whenPressed(new VisionDistanceCalibrate());
		stick2_B.whenPressed(new BallGrabCommand());
		stick2_A.whenPressed(new BallStopCommand());
		stick2_X.whenPressed(new BallDejamCommand());
		stick2_start.whenPressed(new GearAdjustExtendCommand());
		stick2_back.whenPressed(new GearAdjustRetractCommand());
		stick2_RB.whenPressed(new BallDoorOpenCommand());
		stick2_LB.whenPressed(new BallDoorCloseCommand());		
		stick2_dpadRight.whenPressed(new GearDeliveryCommandGroupTelop());				
	}	
}
