package org.usfirst.frc.team63.robot.util;

public class RigidTransform2d{

    public static class Delta {
        public final double dx;
        public final double dy;
        public final double dtheta;

        public Delta(double dx, double dy, double dtheta) {
            this.dx = dx;
            this.dy = dy;
            this.dtheta = dtheta;
        }
    }
}
