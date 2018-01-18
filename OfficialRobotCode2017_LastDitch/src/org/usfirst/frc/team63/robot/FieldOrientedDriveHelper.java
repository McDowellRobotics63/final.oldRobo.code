package org.usfirst.frc.team63.robot;


import org.usfirst.frc.team63.robot.util.DriveSignal;
import org.usfirst.frc.team63.robot.util.SynchronousPID;
import org.usfirst.frc.team63.robot.util.Util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Helper class to implement field-oriented mecanum driving.
 */
public class FieldOrientedDriveHelper {
	private double mOrientationSetpoint = 0.0;
	
    public enum DriveMode {
        OPEN_LOOP, FIELD_ORIENTED;
    }
    
    public enum MotorType {
        kFrontLeft(0), kFrontRight(1), kRearLeft(2), kRearRight(3);

        public final int value;

        private MotorType(int value) {
          this.value = value;
        }
     }
    
    public static final double kThrottleDeadband = 0.15;
    public static final double kRotateDeadband = 0.15;
    private static final double kRotateSensitivity = 1.0;
    private static final double kMaxRotateRate = 180.0; //degress per second
    static final double kToleranceDegrees = 2.0f;
    protected static final int kMaxNumberOfMotors = 4;
    private DriveSignal mSignal = new DriveSignal(0, 0, 0, 0);    
    
    private SynchronousPID turnController;
    private static final double kPTurn = 0.015;
    //private static final double kPTurn = 0.02;
    private static final double kITurn = 0.0;
    //private static final double kITurn = 0.0009;
    private static final double kDTurn = 0.15;    
    
    private DriveMode drive_mode = DriveMode.FIELD_ORIENTED;
    
    private double last_time = -1.0;
    
    public FieldOrientedDriveHelper()
    {
        turnController = new SynchronousPID(kPTurn, kITurn, kDTurn);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setContinuous(true);
    }

    public DriveMode GetDriveMode()
    {
    	return drive_mode;
    }
    
    public void SetOrientationSetpoint(double value)
    {
    	mOrientationSetpoint = value;
    }

    public double GetOrientationSetpoint()
    {
    	return mOrientationSetpoint;
    }
    
    public void ResetOrientationSetpoint()
    {
    	mOrientationSetpoint = Robot.drive.getGyroAngleDegrees();
    }
    
    public DriveSignal fieldOrientedDrive(double fwd_back, double left_right, double rotate, double gyroAngle) {

    	double dt = 0.0;
    	double now = Timer.getFPGATimestamp();
    	// Initialized above as -1.0
    	// The intent here is to measure time between calls to this function.
    	// Want to avoid measuring the time between object creation and first call to this function.
    	if(last_time < 0.0)
    	{
    		dt = 0.0;    		
    	}
    	else
    	{
    		dt = now - last_time;
    	}
    	last_time = now;
    	
    	handleFailedNavX();
    	
        fwd_back = handleDeadband(fwd_back, kThrottleDeadband);
        left_right = handleDeadband(left_right, kThrottleDeadband);
        rotate = handleDeadband(rotate, kRotateDeadband);
        rotate = rotate * kRotateSensitivity;
        
        if(drive_mode != DriveMode.OPEN_LOOP)
        {
        	mOrientationSetpoint = mOrientationSetpoint + (-rotate * kMaxRotateRate * dt);
        	
	        if(mOrientationSetpoint > 179.9)
	        {
	        	mOrientationSetpoint = -179.9;
	        }
	        else if(mOrientationSetpoint < -179.9)
	        {
	        	mOrientationSetpoint = 179.9;
	        }
	        
	    	turnController.setSetpoint(mOrientationSetpoint);
	        rotate = -turnController.calculate(gyroAngle);
        }
        
        if(drive_mode != DriveMode.FIELD_ORIENTED)
        {
        	gyroAngle = 0;
        }
        
        double[] wheelSpeeds = mecanumDrive_Cartesian(left_right, fwd_back, rotate, gyroAngle);
        
        mSignal.leftFrontMotor = wheelSpeeds[MotorType.kFrontLeft.value];
        mSignal.rightFrontMotor = -wheelSpeeds[MotorType.kFrontRight.value];
        mSignal.leftRearMotor = wheelSpeeds[MotorType.kRearLeft.value];
        mSignal.rightRearMotor = -wheelSpeeds[MotorType.kRearRight.value];             
        
        return mSignal;
    }
         
    private void handleFailedNavX()
    {
    	//If NavX poops the bed then stop closing the loop on gyro angle
    	if(!Robot.drive.isNavXConnected())
    	{
    		drive_mode = DriveMode.OPEN_LOOP;
    	}
    	else
    	{
    		//Transition from open loop back to closed loop control
    		if(drive_mode == DriveMode.OPEN_LOOP)
    		{
    			drive_mode = DriveMode.FIELD_ORIENTED;
    			ResetOrientationSetpoint();
    		}
    	}
    }    
    
    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
     * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
     * top, the roller axles should form an X across the robot.
     *
     * <p>This is designed to be directly driven by joystick axes.
     *
     * @param x         The speed that the robot should drive in the X direction. [-1.0..1.0]
     * @param y         The speed that the robot should drive in the Y direction. This input is
     *                  inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
     * @param rotation  The rate of rotation for the robot that is completely independent of the
     *                  translation. [-1.0..1.0]
     * @param gyroAngle The current angle reading from the gyro. Use this to implement field-oriented
     *                  controls.
     */
    private double[] mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
      double xIn = x;
      double yIn = y;
      // Negate y for the joystick.
      yIn = -yIn;
      // Compenstate for gyro angle.
      double[] rotated = Util.rotateVector(xIn, yIn, -gyroAngle);
      xIn = rotated[0];
      yIn = rotated[1];

      double[] wheelSpeeds = new double[kMaxNumberOfMotors];
      wheelSpeeds[MotorType.kFrontLeft.value] = xIn + yIn + rotation;
      wheelSpeeds[MotorType.kFrontRight.value] = -xIn + yIn - rotation;
      wheelSpeeds[MotorType.kRearLeft.value] = -xIn + yIn + rotation;
      wheelSpeeds[MotorType.kRearRight.value] = xIn + yIn - rotation;

      normalize(wheelSpeeds);
     
      return wheelSpeeds;
    }
   
    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    private void normalize(double[] wheelSpeeds) {
      double maxMagnitude = Math.abs(wheelSpeeds[0]);
      for (int i = 1; i < kMaxNumberOfMotors; i++) {
        double temp = Math.abs(wheelSpeeds[i]);
        if (maxMagnitude < temp) {
          maxMagnitude = temp;
        }
      }
      if (maxMagnitude > 1.0) {
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
          wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
        }
      }
    }
}
