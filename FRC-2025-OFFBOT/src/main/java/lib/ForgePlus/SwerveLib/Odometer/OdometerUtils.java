package lib.ForgePlus.SwerveLib.Odometer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class OdometerUtils {

    public static final AprilTagFieldLayout apriltagField = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static boolean isPoseinField(Pose2d pose){

        if (pose.getX() < 0.0 
        || pose.getX() > apriltagField.getFieldLength() 
        || pose.getY() < 0.0 
        || pose.getY() > apriltagField.getFieldWidth()) {

            return false;
        } else {
            return true;
        }
    }

    /**
     * Flips the given {@code Pose2d} around the field, inverting its Y-coordinate relative to the field's width.
     * The rotation is also modified by adjusting its angle using sine and cosine components.
     * 
     * @param pose The {@code Pose2d} to be flipped.
     * @param field The {@code FieldObject} representing the field's dimensions (specifically its width).
     * @return A new {@code Pose2d} representing the flipped pose.
     */
    public static Pose2d flipPose(Pose2d pose, FieldObject field){
        return new Pose2d(pose.getX(), field.width() - pose.getY(), new Rotation2d(pose.getRotation().getCos() -pose.getRotation().getSin()));
    }

    public static boolean isBlue(){

        return DriverStation.getAlliance().
        orElse(Alliance.Blue) == Alliance.Blue;

    }

    public static boolean isRed(){
        return !isBlue();
    }

    
}
