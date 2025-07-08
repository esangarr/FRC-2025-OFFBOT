package frc.robot.DriveCommands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriveTrain.Swerve;
import lib.ForgePlus.Math.Profiles.Control.MotionModelControl;
import lib.ForgePlus.Math.Profiles.Control.PositionState;
import lib.ForgePlus.Math.Profiles.ProfileGains.MotionModelGains;
import lib.ForgePlus.SwerveLib.Odometer.OdometerUtils;

public class DriveCommands {

    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    public static Command joystickDrive(
        Swerve drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier){
    return Commands.run(
        () -> {
            // Apply deadband
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square values
            omega = Math.copySign(omega * omega, omega);

            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec());

            drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  OdometerUtils.isRed()
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
    }

    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
  
    public static Command joystickSnapAngle(
        Swerve drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Rotation2d> angleSupplier) {

        MotionModelControl angleController = new MotionModelControl(
            new MotionModelGains(
                ANGLE_KP,
                0,
                ANGLE_KD,
                ANGLE_MAX_VELOCITY,
                ANGLE_MAX_ACCELERATION));

        angleController.continuousInput(-Math.PI, Math.PI);

        return Commands.run(
            () -> {

                Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                double omega =
                    angleController.calculate(
                        new PositionState(angleSupplier.get().getRadians()),
                        drive.getRotation().getRadians()
                    ).getOutput();

                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            OdometerUtils.isRed()
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));

            },
            drive)
            
            .beforeStarting(()-> angleController.reset(drive.getRotation().getRadians(), 0));
    }

}
