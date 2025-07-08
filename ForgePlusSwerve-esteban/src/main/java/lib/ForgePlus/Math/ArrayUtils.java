package lib.ForgePlus.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArrayUtils {
    
    public static double[] toArray(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public static double[] toArray(Transform2d transform) {
        return new double[] {transform.getX(), transform.getY(), transform.getRotation().getRadians()};
    }

    public static double[] toArray(Translation2d translation) {
        return new double[] {translation.getX(), translation.getY()};
    }

}
