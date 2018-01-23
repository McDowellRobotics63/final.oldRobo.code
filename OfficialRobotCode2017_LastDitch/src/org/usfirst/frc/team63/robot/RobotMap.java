package org.usfirst.frc.team63.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // Wheels
	public static final double kDriveWheelDiameterInches = 5.5963;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2;
	// Distance left to right wheel / 2
	public static final double kWheelSeparationWidth = 14.0;
	// Distance front to back wheel / 2
	public static final double kWheelSeparationLength = 14.0;
	
	//Config params for closed loop position control

	public static final int kTimeoutMs = 10;
	public static final int pidIndex = 0;
	public static final double kP_position_strafe = 0.02;
	public static final double kI_position_strafe = 0.0;
	public static final double kD_position_strafe = 0.0;
	public static final double creep_speed_position_strafe = 0.5;
	public static final double max_speed_position_strafe = 0.8;

	public static final double kP_position = 0.01;
	public static final double kI_position = 0.0;
	public static final double kD_position = 0.0;
	public static final double creep_speed_position = 0.2;
	public static final double max_speed_position = 0.6; //.4
	public static final double ramp_rate_position = 1.0; //.8
	public static final double max_distance_position = 600.0; // inches
	public static final double kTolerance_position = 1; //inches
	public static final double kOnTargetTime_position = 0.25; //seconds must be within position tolerance
	//End config params for close loop position control
	
	//Config params for closed loop heading control
	public static final double kP_rotate = 4.0;
	public static final double kI_rotate = 0.0;
	public static final double kD_rotate = 0.0;
	public static final double creep_speed_rotate = (creep_speed_position / kWheelSeparationWidth) * (180.0f / Math.PI); //degrees per second, i.e. minimum speed
	public static final double max_speed_rotate = 120.0; // degrees per second, max command speed
	public static final double max_degrees_rotate = 180.0; // degrees
	public static final double kTolerance_rotate = 2; //degrees
	public static final double kOnTargetTime_rotate = 0.25; //seconds must be within degrees tolerance
	//End config params for close loop heading control
	
    public static final int AUTO_SWITCH_1 = 1;
    public static final int AUTO_SWITCH_2 = 2;
    public static final int AUTO_SWITCH_3 = 3;
	
	public static final int XBOX_LEFT_X_AXIS = 0;
    public static final int XBOX_LEFT_Y_AXIS = 1;
    public static final int XBOX_LEFT_TRIGGER_AXIS = 2;
    public static final int XBOX_RIGHT_TRIGGER_AXIS = 3;
    public static final int XBOX_RIGHT_X_AXIS = 4;
    public static final int XBOX_RIGHT_Y_AXIS = 5;
    public static final int XBOX_A = 1;
    public static final int XBOX_B = 2;
    public static final int XBOX_X = 3;
    public static final int XBOX_Y = 4;
    public static final int XBOX_LEFT_BUMPER = 5;    
    public static final int XBOX_RIGHT_BUMPER = 6;
    public static final int XBOX_BACK = 7;
    public static final int XBOX_START = 8;
    
	//DRIVE MAP
	public static final int FRONT_RIGHT_MOTOR = 4;
	public static final int FRONT_LEFT_MOTOR = 1;
	public static final int REAR_RIGHT_MOTOR = 3;
	public static final int REAR_LEFT_MOTOR = 2;
	
	public static final int FRONT_LEFT_ENCODER = 0;
	public static final int FRONT_RIGHT_ENCODER = 0;
	public static final int BACK_LEFT_ENCODER = 0;
	public static final int BACK_RIGHT_ENCODER = 0;
	
	public static double kDriveVelocityKpTelop = 1.6153;
    public static double kDriveVelocityKiTelop = 0.016153;
    public static double kDriveVelocityKdTelop = 0.0;
    public static double kDriveVelocityKfTelop = 1.4548;
	
	public static double kDriveVelocityKpAuto = 0.0533;
    public static double kDriveVelocityKiAuto = 0.00533;
    public static double kDriveVelocityKdAuto = 0.533;
    public static double kDriveVelocityKfAuto = 0.1647;
    
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;
    public static final int kVelocityControlSlot = 0;
    
    public static final double kCreepGearForwardSpeed = 10.0;
    public static final double kCreepGearBackSpeed = 10.0;
    public static final double kCreepGearLeftSpeed = 5.0;
    public static final double kCreepGearRightSpeed = 5.0;   


    
    public static double kCameraResWidth = 640;
    public static double kCameraResHeight = 320;
    public static double kCameraFieldOfViewTheta = 68.5/2.0;

    
	
	//CLIMB MECHANISM MAP
	public static final int ROBOT_CLIMB_MOTOR = 5;
	
	//BALL MECHANISM MAP
	public static final int BALL_GRAB_MOTOR = 1;
	public static final int BALL_TICKLER_MOTOR = 2;
	public static final int BALL_HOLDER_SOLENOID = 0;

	//SOLENOID MAP
	public static final int GEAR_ADJUST_RETRACT_SOLENOID = 0;
	public static final int GEAR_ADJUST_EXTEND_SOLENOID = 1;
	public static final int GEAR_DOOR_DROP_SOLENOID = 3;
	public static final int GEAR_DOOR_RAISE_SOLENOID = 2;
	public static final int GEAR_PUSHER_RETRACT_SOLENOID = 4;
	public static final int GEAR_PUSHER_EXTEND_SOLENOID = 5;
	public static final int BALL_DOOR_OPEN_SOLENOID = 6;
	public static final int BALL_DOOR_CLOSE_SOLENOID = 7;

}
