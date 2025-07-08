package lib.ForgePlus.SwerveLib.Odometer;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles pose estimation for a swerve drive using WPILib's {@link SwerveDrivePoseEstimator}.
 * <p>
 * <b>Note:</b> Only one instance of this class should be created. Creating multiple instances
 * may lead to unexpected behavior or incorrect pose estimation.
 */
public class ForgeSwerveDrivePoseEstimator {
    
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroRotation;
    private BooleanSupplier gyroConnection;

    private Notifier backgroundUpdater;

    private Supplier<SwerveModulePosition[]> getModulePositions;
    private Supplier<Rotation2d> getGyroRotation;

    private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

    private Field2d field = null;

    public static final double kDefaultPeriod = 0.02;

    /**
     * Creates a new instance of {@link ForgeSwerveDrivePoseEstimator}.
     * <p>
     * <b>Warning: </b> Only one instance should be created. Running multiple instances
     * simultaneously may cause conflicts in pose estimation.
     *
     * @param kinematics        The swerve drive kinematics model.
     * @param gyroConnection    Supplier that indicates if the gyro is connected.
     * @param getModulePositions Supplier for swerve module positions.
     * @param getGyroRotation   Supplier for gyro rotation readings.
     * @param createFieldWidget Whether to create a Field2d dashboard widget.
     */
    public ForgeSwerveDrivePoseEstimator(
        SwerveDriveKinematics kinematics,
        BooleanSupplier gyroConnection,
        Supplier<SwerveModulePosition[]> getModulePositions,
        Supplier<Rotation2d> getGyroRotation,
        boolean createFieldWidget) {

        this.kinematics = kinematics;
        this.gyroConnection = gyroConnection;
        this.rawGyroRotation = new Rotation2d();

        this.getModulePositions = getModulePositions;
        this.getGyroRotation = getGyroRotation;

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            Pose2d.kZero);

        if (createFieldWidget == true) {
            this.field = new Field2d();
            SmartDashboard.putData("ForgePoseEstimator/Field", field);
        }
        
        this.backgroundUpdater = new Notifier(this::update);
        backgroundUpdater.startPeriodic(kDefaultPeriod);

    }

    /**
     * Creates a new instance of {@link ForgeSwerveDrivePoseEstimator}.
     * <p>
     * <b>Warning: </b> Only one instance should be created. Running multiple instances
     * simultaneously may cause conflicts in pose estimation.
     *
     * @param kinematics        The swerve drive kinematics model.
     * @param gyroConnection    Supplier that indicates if the gyro is connected.
     * @param getModulePositions Supplier for swerve module positions.
     * @param getGyroRotation   Supplier for gyro rotation readings.
     * @param createFieldWidget Whether to create a Field2d dashboard widget.
     * @param period The period to update the pose Estimator
     */
    public ForgeSwerveDrivePoseEstimator(
        SwerveDriveKinematics kinematics,
        BooleanSupplier gyroConnection,
        Supplier<SwerveModulePosition[]> getModulePositions,
        Supplier<Rotation2d> getGyroRotation,
        boolean createFieldWidget,
        double period) {

        this.kinematics = kinematics;
        this.gyroConnection = gyroConnection;
        this.rawGyroRotation = new Rotation2d();

        this.getModulePositions = getModulePositions;
        this.getGyroRotation = getGyroRotation;

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            Pose2d.kZero);

        if (createFieldWidget == true) {
            this.field = new Field2d();
            SmartDashboard.putData("ForgePoseEstimator/Field", field);
        }
        
        this.backgroundUpdater = new Notifier(this::update);
        this.backgroundUpdater.setName("ForgePoseEstimatorUpdater");
        backgroundUpdater.startPeriodic(period);

    }

    private void update() {

        SwerveModulePosition[] modulePositions = getModulePositions.get();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }
        
        if (gyroConnection.getAsBoolean() == true) {
            rawGyroRotation = getGyroRotation.get();
        } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(rawGyroRotation, modulePositions);

        if (field != null) {
            field.setRobotPose(getEstimatedPose()); 
        }      
    }

    /**
     * Gets the estimated pose of the robot.
     *
     * @return The current estimated {@link Pose2d}.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Optional<Field2d> getFieldWidget(){

        if (field == null) {
            return Optional.empty();
        }

        return Optional.of(field);
    }

    public void addObjectToFieldWidget(String name, Pose2d objectPose) {
        if (field != null) {
            field.getObject(name).setPose(objectPose);
        }
    }

    public void addFieldObjectsToFieldWidget(String name, Pose2d... objectPoses) {
        if (field != null) {
            field.getObject(name).setPoses(objectPoses);
        }
    }

    /**
     * Resets the pose estimator to a specified position.
     *
     * @param pose The new pose to set.
     */
    public void resetPosition(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions.get(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionMeasurement The estimated pose from vision.
     * @param timestamp The timestamp of the measurement.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionMeasurement The estimated pose from vision.
     * @param timestamp The timestamp of the measurement.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement,timestamp, visionMeasurementStdDevs);
    }

     /**
     * Stops background updates from running.
     */
    public void stopUpdates(){
        backgroundUpdater.stop();
    }

}
