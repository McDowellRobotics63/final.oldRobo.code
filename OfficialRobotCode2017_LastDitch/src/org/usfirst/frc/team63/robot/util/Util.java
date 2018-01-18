package org.usfirst.frc.team63.robot.util;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    /**
     * Rotate a vector in Cartesian space.
     */
    public static double[] rotateVector(double x, double y, double angle) {
      double cosA = Math.cos(angle * (3.14159 / 180.0));
      double sinA = Math.sin(angle * (3.14159 / 180.0));
      double[] out = new double[2];
      out[0] = x * cosA - y * sinA;
      out[1] = x * sinA + y * cosA;
      return out;
    }
}
