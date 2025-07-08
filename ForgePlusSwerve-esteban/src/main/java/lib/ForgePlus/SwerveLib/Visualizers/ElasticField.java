package lib.ForgePlus.SwerveLib.Visualizers;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;

public class ElasticField extends SubsystemBase{
    private final Field2d object;
    private Supplier<Pose2d> robotPose;

    public ElasticField(String key, String name, Supplier<Pose2d> poseSupplier) {
        object = new Field2d();
        NTPublisher.publish(key, name, object);
        this.robotPose = poseSupplier;
    }

    @Override
    public void periodic(){
        if (robotPose != null) {
            object.setRobotPose(robotPose.get());
        }
    }

    public Field2d getField() {
        return object;
    }

}
