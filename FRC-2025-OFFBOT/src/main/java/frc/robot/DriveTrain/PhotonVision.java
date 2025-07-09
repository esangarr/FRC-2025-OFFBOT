package frc.robot.DriveTrain;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;

public class PhotonVision extends NetworkSubsystem{

    PhotonCamera camera = new PhotonCamera("Arducam1");

    public PhotonVision() {
        super("PhotonVision/Data", false);
    }

    @Override
    public void NetworkPeriodic(){}

    @AutoNetworkPublisher(key = "HasTarget?")
    public boolean hasTarget(){
        var result = camera.getLatestResult();

        return result.hasTargets();
    }

    @AutoNetworkPublisher(key = "getBestTaget")
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


  
   
}
