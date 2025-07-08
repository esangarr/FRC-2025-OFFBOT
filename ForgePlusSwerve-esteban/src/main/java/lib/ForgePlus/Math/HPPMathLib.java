package lib.ForgePlus.Math;

import edu.wpi.first.math.util.Units;

public class HPPMathLib {

    public static double coterminalDegrees(double angle){
        if (angle >= 0 && angle < 360) {
            return angle;
        }
        else if (angle > 360) {
            return angle - Math.floor(angle / 360) * 360;
        }
        else {
            return angle + (1 + Math.floor(-angle / 360)) * 360;
        }

    }

    public static double coterminalRadians(double angle){
        if (angle >= 0 && angle < 2 * Math.PI) {
            return angle;
        }
        else if (angle >  2 * Math.PI) {
            return angle - Math.floor(angle /  (2 * Math.PI)) *  (2 * Math.PI);
        }
        else {
            return angle + (1 + Math.floor(-angle /  (2 * Math.PI))) *  (2 * Math.PI);
        }
    }

    public static double MinAngleDegrees(double ang_from, double ang_to) {
        if (Math.abs(ang_to - ang_from) < coterminalDegrees(ang_to - ang_from)) {
            return ang_to - ang_from;
        }
        else {
            return coterminalDegrees(ang_to - ang_from);
        }
    }
    public static double MinAngleRadians(double ang_from, double ang_to) {
        if (Math.abs(ang_to - ang_from) < coterminalRadians(ang_to - ang_from)) {
            return ang_to - ang_from;
        }
        else {
            return coterminalRadians(ang_to - ang_from);
        }
    }

    public static double atan3Radians(double x, double y, double threshold, double defaultValue) {
        if (Math.abs(x * x + y * y) < threshold) {
            return defaultValue;
        }

        return Math.atan2(y, x);

    }

    public static double atan3Degrees(double x, double y, double threshold, double defaultValue) {
        return Units.radiansToDegrees(atan3Radians(x, y, threshold, defaultValue));

    }
}
