package org.usfirst.frc.team63.robot.util;

public class DriveVelocity {
    public double left_front;
    public double left_rear;
    public double right_front;
    public double right_rear;

    public DriveVelocity(double left_front, double left_rear, double right_front, double right_rear) {
        this.left_front = left_front;
        this.left_rear = left_rear;
        this.right_front = right_front;
        this.right_rear = right_rear;
    }
}
