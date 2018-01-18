package org.usfirst.frc.team63.robot;

import org.usfirst.frc.team63.robot.util.DriveVelocity;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;

public class Kinematics {
	
    public static RigidTransform2d.Delta forwardKinematics(double left_front_velocity, double left_rear_velocity,
												           double right_front_velocity, double right_rear_velocity) {
    	double left = left_front_velocity + left_rear_velocity;
    	double right = right_front_velocity + right_rear_velocity;
		double dx = (right + left)/2;
		double dy = 0;
		double dtheta = (right - left)/RobotMap.kWheelSeparationWidth;
		return new RigidTransform2d.Delta(dx, dy, dtheta);
    }
     
    public static RigidTransform2d.Delta forwardKinematics(double left_front_velocity, double left_rear_velocity,
	           double right_front_velocity, double right_rear_velocity,
	           double delta_radians) {
    	double left = left_front_velocity + left_rear_velocity;
    	double right = right_front_velocity + right_rear_velocity;
    	double dx = (left+right)/2;
    	double dy = 0;
    	double dtheta = delta_radians;
    	return new RigidTransform2d.Delta(dx, dy, dtheta);
}
   
    /**
     * Inverse kinematics to obtain the individual wheel velocities needed to achieve a desired robot motion
     */
    public static DriveVelocity inverseKinematics(RigidTransform2d.Delta velocity) {    	
    	
//    	//convert dtheta from degrees per second to inches per second
//    	double rotation_inches_per_sec = velocity.dtheta * RobotMap.kWheelSeparationWidth * Math.PI / 180.0f;
//    	
//    	double left_front = (velocity.dx - velocity.dy - rotation_inches_per_sec);
//
//		double right_front = (velocity.dx + velocity.dy + rotation_inches_per_sec);
//		
//		double left_rear = (velocity.dx + velocity.dy - rotation_inches_per_sec);
//		
//		double right_rear = (velocity.dx - velocity.dy + rotation_inches_per_sec);
//    	
//    	return new DriveVelocity(left_front, left_rear, right_front, right_rear);
    	
    	double left = velocity.dx*2 - velocity.dtheta*RobotMap.kWheelSeparationWidth;

		double right = velocity.dx*2 + velocity.dtheta*RobotMap.kWheelSeparationWidth;
    	
    	return new DriveVelocity(left, left, right, right);
    }
}
