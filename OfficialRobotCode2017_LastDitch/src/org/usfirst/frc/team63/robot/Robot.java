
package org.usfirst.frc.team63.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team63.robot.autonomous_routines.AutoBallsThenGearDeliverMiddle;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoBallsThenGearDeliverRight;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoToGreenLineThenBalls;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoDriveToGreenLine;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoGearDeliver;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoGearDeliver.GearDeliverPosition;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoToGreenLineThenBalls.AllianceColor;
import org.usfirst.frc.team63.robot.autonomous_routines.AutoGearDeliverMashup;
import org.usfirst.frc.team63.robot.commands.VisionDistanceCalibrate;
import org.usfirst.frc.team63.robot.simple_commands.GearAdjustExtendCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearAdjustRetractCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearDoorDropCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearDoorRaiseCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearPusherExtendCommand;
import org.usfirst.frc.team63.robot.simple_commands.GearPusherRetractCommand;
import org.usfirst.frc.team63.robot.subsystems.BallMechanismSubsystem;
import org.usfirst.frc.team63.robot.subsystems.ClimbSubsystem;
import org.usfirst.frc.team63.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team63.robot.subsystems.GearMechanismSubsystem;
import org.usfirst.frc.team63.robot.subsystems.VisionSubsystem;

//This is a change
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
		
	public static OI oi;
	public static final BallMechanismSubsystem balls = new BallMechanismSubsystem();
	public static final ClimbSubsystem climb = new ClimbSubsystem();
	public static final DriveSubsystem drive = new DriveSubsystem();
	public static final GearMechanismSubsystem gears = new GearMechanismSubsystem();
	public static VisionSubsystem vision;
    DigitalInput switch1 = new DigitalInput(RobotMap.AUTO_SWITCH_1);
    DigitalInput switch2 = new DigitalInput(RobotMap.AUTO_SWITCH_2);
    DigitalInput switch3 = new DigitalInput(RobotMap.AUTO_SWITCH_3);

	
	Command autonomousCommand;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		vision = new VisionSubsystem();
		oi = new OI();http:
		
		SmartDashboard.putNumber("vision_test_creep_speed", 5);
		SmartDashboard.putNumber("vision_test_end_flag", 0);
		SmartDashboard.putData("Gear Door Open", new GearDoorDropCommand());
		SmartDashboard.putData("Gear Door Close", new GearDoorRaiseCommand());
		SmartDashboard.putData("Gear Pusher Extend", new GearPusherExtendCommand());
		SmartDashboard.putData("Gear Pusher Retract", new GearPusherRetractCommand());
		SmartDashboard.putData("Gear Adjuster Extend", new GearAdjustExtendCommand());
		SmartDashboard.putData("Gear Adjuster Retract", new GearAdjustRetractCommand());
		SmartDashboard.putData("Vision Distance Calibrate", new VisionDistanceCalibrate());
		SmartDashboard.putNumber("TicklerVal", 0);
		SmartDashboard.putNumber("BallLiftVal", -0.35);
		
		//CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
        Robot.drive.setTalonBaseConfigurationAuto();
        Robot.drive.setTalonBaseConfigurationAuto();
        Robot.drive.setTalonBaseConfigurationAuto();
        Robot.drive.setTalonBaseConfigurationAuto();
		
		System.out.println("autonomousInit()");
		
    	Robot.drive.zeroSensors();   	
    	Robot.drive.mDriveHelper.ResetOrientationSetpoint();
    	
    	if (switch1.get() == true && switch2.get() == false && switch3.get() == false)
    	{
    		System.out.println("Running AutoDriveToGreenLine()");
    		autonomousCommand = new AutoDriveToGreenLine();
    	}    	
    	else if(switch1.get() == false && switch2.get() == true && switch3.get() == false)
		{
    		autonomousCommand = new AutoGearDeliverMashup();
		}
    	else if(switch1.get() == false && switch2.get() == false && switch3.get() == true)
    	{
    		autonomousCommand = new AutoToGreenLineThenBalls(AllianceColor.RED_ALLIANCE);
    	}
    	else if(switch1.get() == true && switch2.get() == true && switch3.get() == false)
    	{
    		autonomousCommand = new AutoToGreenLineThenBalls(AllianceColor.BLUE_ALLIANCE);
    	}
    	else if(switch1.get() == false && switch2.get() == true && switch3.get() == true)
    	{
    		//autonomousCommand = new AutoBallsThenToGreenLine();
    	}
    	else if(switch1.get() == true && switch2.get() == false && switch3.get() == true)
    	{
    		//autonomousCommand = new AutoBallsThenGearDeliverRight();
    	}
    	else if(switch1.get() == true && switch2.get() == false && switch3.get() == true)
    	{
    		//autonomousCommand = new AutoBallsThenGearDeliverMiddle();
    	}
		else
		{
			autonomousCommand = null;
		}
    	
        if (autonomousCommand != null)
        {
        	System.out.println("Starting auto command");
        	autonomousCommand.start();
        }
        else
        {
        	System.out.println("Auto command is null!!!");
        }
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		Robot.drive.outputToSmartDashboard();
	}

	@Override
	public void teleopInit() {
        Robot.drive.setTalonBaseConfigurationTelop();
        Robot.drive.setTalonBaseConfigurationTelop();
        Robot.drive.setTalonBaseConfigurationTelop();
        Robot.drive.setTalonBaseConfigurationTelop();
        
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		Robot.drive.mDriveHelper.ResetOrientationSetpoint();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		Robot.drive.outputToSmartDashboard();
		SmartDashboard.putBoolean("Auto_Switch_1", switch1.get());
		SmartDashboard.putBoolean("Auto_Switch_2", switch2.get());
		SmartDashboard.putBoolean("Auto_Switch_3", switch3.get());   
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
