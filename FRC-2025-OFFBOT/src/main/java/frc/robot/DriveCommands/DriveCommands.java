package frc.robot.DriveCommands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriveTrain.Swerve;
//import frc.robot.DriveTrain.Swerve;
import lib.ForgePlus.Math.Profiles.Control.MotionModelControl;
import lib.ForgePlus.Math.Profiles.Control.PositionState;
import lib.ForgePlus.Math.Profiles.ProfileGains.MotionModelGains;
import lib.ForgePlus.REV.REVBlinkin.REVBlinkin.PatternType;
import lib.ForgePlus.SwerveLib.Odometer.OdometerUtils;

public class DriveCommands {

    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    public static Command setTurnAngle( Swerve drive, Rotation2d angle){

        return Commands.run(()-> {
            drive.modules[0].setTurnPos(angle);
            drive.modules[1].setTurnPos(angle);
            drive.modules[2].setTurnPos(angle);
            drive.modules[3].setTurnPos(angle);
        }, drive);

    }

    public static Command setVelocity(Swerve drive, double velocity){
        return Commands.run(()-> {
            drive.modules[0].setDriveVelocity(velocity);
            drive.modules[1].setDriveVelocity(velocity);
            drive.modules[2].setDriveVelocity(velocity);
            drive.modules[3].setDriveVelocity(velocity);
        });
    }

    

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

    public static Command joystickDrive(Swerve drive,DoubleSupplier xSupplier,DoubleSupplier ySupplier,DoubleSupplier omegaSupplier){
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

    public static Command brake(Swerve drive){
        return Commands.run(()->{drive.stopWithX();}, drive);
    }

    public static Command moveInX(Swerve drive, double speed){
        return Commands.run(()-> {
            drive.runVelocity(new ChassisSpeeds(0, speed, 0));
        }, drive);
     }

    public static Command moveInY(Swerve drive, double speed){
        return Commands.run(()-> {
            drive.runVelocity(new ChassisSpeeds(speed,0, 0));}, drive);
    }   

    public static Command resetHeading(Swerve drive){
        return Commands.runOnce(()-> {drive.resetHeading();});
    }

    private static final double FF_START_DELAY = 2.0; 
    private static final double FF_RAMP_RATE = 0.1;

    public static Command feedforwardCharacterization(Swerve drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                velocitySamples.clear();
                voltageSamples.clear();
              }),

             // Allow modules to orient
            Commands.run(
                    () -> {
                    drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterization(voltage);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                    int n = velocitySamples.size();
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumXY = 0.0;
                    double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                    }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  SmartDashboard.putString("ks", formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  SmartDashboard.putString("kV", formatter.format(kV));
                }));
  }



}
