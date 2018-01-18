package org.usfirst.frc.team63.robot;

import org.usfirst.frc.team63.robot.util.DriveVelocity;
import org.usfirst.frc.team63.robot.util.RigidTransform2d;

public class Kinematics {
	
    public static RigidTransform2d.Delta forwardKinematics(double left_front_velocity, double left_rear_velocity,
												           double right_front_velocity, double right_rear_velocity,
												           double delta_radians) {
		double dx = (left_front_velocity + right_front_velocity + left_rear_velocity + right_rear_velocity) / 4;
		double dy = (-left_front_velocity + right_front_velocity + left_rear_velocity - right_rear_velocity) / 4;
		double dtheta = (-left_front_velocity + right_front_velocity - left_rear_velocity + right_rear_velocity) /
		(4 * (RobotMap.kWheelSeparationWidth + RobotMap.kWheelSeparationLength));
		
		dtheta = delta_radians;
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
    	
    	double left_front = (velocity.dx - velocity.dy - velocity.dtheta);

		double right_front = (velocity.dx + velocity.dy + velocity.dtheta);
		
		double left_rear = (velocity.dx + velocity.dy - velocity.dtheta);
		
		double right_rear = (velocity.dx - velocity.dy + velocity.dtheta);
    	
    	return new DriveVelocity(left_front, left_rear, right_front, right_rear);
    }
}
