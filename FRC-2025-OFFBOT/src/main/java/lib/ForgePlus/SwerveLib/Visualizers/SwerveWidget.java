package lib.ForgePlus.SwerveLib.Visualizers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.SwerveLib.Utils.SwerveModuleStateSupplier;

/**
 * A utility class for visualizing swerve module states on SmartDashboard.
 * This class builds a SmartDashboard widget to display module angles, velocities, and robot rotation.
 */
public class SwerveWidget {

    private SwerveWidget(){}

    /**
     * Creates a SmartDashboard visualization for a swerve drivetrain.
     *
     * @param keyName The SmartDashboard key name for the widget.
     * @param FrontLeft The front-left swerve module state supplier.
     * @param FrontRight The front-right swerve module state supplier.
     * @param BackLeft The back-left swerve module state supplier.
     * @param BackRight The back-right swerve module state supplier.
     * @param RobotRotation A DoubleSupplier providing the robot's rotation angle.
     */
    public static void build(
        String keyName,
        SwerveModuleStateSupplier FrontLeft,
        SwerveModuleStateSupplier FrontRight,
        SwerveModuleStateSupplier BackLeft,
        SwerveModuleStateSupplier BackRight,
        DoubleSupplier RobotRotation){
    
        SmartDashboard.putData(keyName, new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", FrontLeft.angle(), null);
        builder.addDoubleProperty("Front Left Velocity", FrontLeft.speedMetersPerSecond(), null);

        builder.addDoubleProperty("Front Right Angle", FrontRight.angle(), null);
        builder.addDoubleProperty("Front Right Velocity", FrontRight.speedMetersPerSecond(), null);

        builder.addDoubleProperty("Back Left Angle", BackLeft.angle(), null);
        builder.addDoubleProperty("Back Left Velocity", FrontRight.speedMetersPerSecond(), null);

        builder.addDoubleProperty("Back Right Angle", BackRight.angle(), null);
        builder.addDoubleProperty("Back Right Velocity", BackRight.speedMetersPerSecond(), null);

        builder.addDoubleProperty("Robot Angle", RobotRotation, null);
        }
        });

    }

    public static void buildCustomPath(
        String table,
        String key,
        SwerveModuleStateSupplier FrontLeft,
        SwerveModuleStateSupplier FrontRight,
        SwerveModuleStateSupplier BackLeft,
        SwerveModuleStateSupplier BackRight,
        DoubleSupplier RobotRotation) {

        Sendable widget = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", FrontLeft.angle(), null);
                builder.addDoubleProperty("Front Left Velocity", FrontLeft.speedMetersPerSecond(), null);

                builder.addDoubleProperty("Front Right Angle", FrontRight.angle(), null);
                builder.addDoubleProperty("Front Right Velocity", FrontRight.speedMetersPerSecond(), null);

                builder.addDoubleProperty("Back Left Angle", BackLeft.angle(), null);
                builder.addDoubleProperty("Back Left Velocity", BackLeft.speedMetersPerSecond(), null);

                builder.addDoubleProperty("Back Right Angle", BackRight.angle(), null);
                builder.addDoubleProperty("Back Right Velocity", BackRight.speedMetersPerSecond(), null);

                builder.addDoubleProperty("Robot Angle", RobotRotation, null);
            }
        };

        NTPublisher.publish(table, key, widget);
    }
}
