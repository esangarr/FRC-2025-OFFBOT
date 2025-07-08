package frc.robot.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.QuestNav;
import lib.ForgePlus.Math.StandardDeviations;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class OculusPlus extends NetworkSubsystem{

    private final QuestNav nav;
    //private final Transform2d robotToQuest;
    public final StandardDeviations dev = new StandardDeviations(0.02, 0.02, 0.035);

    private static final Transform2d robotToQuest = new Transform2d(-0.3684, 0.0465, Rotation2d.k180deg);

    public OculusPlus() {
        super("QuestNav/FORGE", false);
        this.nav = new QuestNav();
        //this.robotToQuest = transform;
    }

    @AutoNetworkPublisher(key = "BatteryPercent")
    public double getBatteryPercent(){
        return nav.getBatteryPercent();
    }

    public Pose2d getPose(){
        return nav.getPose().transformBy(robotToQuest.inverse());
    }
    

    public void setPoseQuest(Pose2d pose){
        Pose2d questPose = pose.transformBy(robotToQuest);
        nav.setPose(questPose);
    }

    public void resetPose(){
        setPoseQuest(new Pose2d());
    }

    @Override
    public void NetworkPeriodic(){
        
    }

    public void update(){
        nav.commandPeriodic();
    }

    @AutoNetworkPublisher(key = "hasPose")
    public boolean hasPose(){
        return nav.isConnected() && nav.isTracking();
    }
    @AutoNetworkPublisher(key = "GetTimeStamp")
    public double timestamp(){
        return nav.getDataTimestamp();
    }

}
