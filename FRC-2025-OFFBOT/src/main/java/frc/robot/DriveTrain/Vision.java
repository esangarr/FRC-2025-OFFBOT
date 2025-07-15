package frc.robot.DriveTrain;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;

public class Vision extends NetworkSubsystem{

    PhotonCamera camera ;

    private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private final PhotonPoseEstimator photonEstimator ;
    private final EstimateConsumer estConsumer;

    private Matrix<N3, N1> curStdDevs;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);// Valores falsos (hay que sacarlos nosotros)
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);// Valores falsos (hay que sacarlos nosotros)

    private boolean target = false;

    public Vision(String cameraName , Transform3d kRobotToCam, EstimateConsumer estConsumer) {
        
        super("photonForge/Data", false);


        this.camera = new PhotonCamera(cameraName);
        this.photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        this.estConsumer = estConsumer;

        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }

    @Override
    public void NetworkPeriodic(){

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            target = !visionEst.isEmpty();

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        publishOutput("Pose", est.estimatedPose.toPose2d());

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }

        
    }

    @AutoNetworkPublisher(key = "Target")
    public boolean hasTarget(){
        return target;
    }

    public PhotonTrackedTarget getBestTarget(){
        var result = camera.getLatestResult();

        return result.getBestTarget();
    }

    public double getYaw(){
        return getBestTarget().getYaw();
    }

    public double getPitch(){
        return getBestTarget().getPitch();
    }

    public double getArea(){
        return getBestTarget().getArea();
    }

    public double getSkew(){
        return getBestTarget().getSkew();
    }

    public Transform2d getCameraToTarget2D() {
        PhotonTrackedTarget target = getBestTarget();
        if (target == null) return null;

        Transform3d camToTarget3d = target.getBestCameraToTarget();
        Transform2d camToTarget2d = new Transform2d(
            camToTarget3d.getTranslation().toTranslation2d(),
            camToTarget3d.getRotation().toRotation2d());

        return camToTarget2d;
    }

    
    public Transform3d getCameraToTarget3D(){
        return getBestTarget().getBestCameraToTarget();
    }

    public List<TargetCorner> getCorners(){
        return getBestTarget().getDetectedCorners();
    }

    public int getID(){
        return getBestTarget().getFiducialId();
    }

    public double getPoseAmbiguity(){
        return getBestTarget().getPoseAmbiguity();
    }
    
    public List<PhotonTrackedTarget> getTarget(){
        var result = camera.getLatestResult();

        return result.getTargets();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
   
}
