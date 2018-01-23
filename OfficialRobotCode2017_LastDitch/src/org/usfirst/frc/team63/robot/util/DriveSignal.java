package org.usfirst.frc.team63.robot.util;

/**
 * A drivetrain command consisting of the four motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal {
    public double leftFrontMotor;
    public double rightFrontMotor;
    public double leftRearMotor;
    public double rightRearMotor;
    public boolean breakMode;

    public DriveSignal(double leftFront, double leftRear, double rightFront, double rightRear) {
        this(leftFront, leftRear, rightFront, rightRear, false);
    }

    public DriveSignal(double leftFront, double leftRear, double rightFront, double rightRear, boolean breakMode) {
        this.leftFrontMotor = leftFront;
        this.leftRearMotor = leftRear;
        this.rightFrontMotor = rightFront;
        this.rightRearMotor = rightRear;
        this.breakMode = breakMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, 0, 0);
    public static DriveSignal BREAK = new DriveSignal(0, 0, 0, 0, true);

    @Override
    public String toString() {
        return "LF: " + leftFrontMotor + ", LR: " + leftRearMotor + ", RF: " + rightFrontMotor + ", RR: " + rightRearMotor;
    }
}