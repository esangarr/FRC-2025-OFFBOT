package frc.robot.DriveTrain;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.proto.Pose2dProto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.PubSubOption;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.protos.generated.Data;
import gg.questnav.questnav.protos.wpilib.DeviceDataProto;
import gg.questnav.questnav.protos.wpilib.FrameDataProto;
import lib.ForgePlus.Math.StandardDeviations;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;



public class OculusPlus extends NetworkSubsystem{

    private final QuestNav nav;

    public final StandardDeviations dev = new StandardDeviations(0.02, 0.02, 0.035);

    public final static Transform2d robotToQuest = new Transform2d(-0.3684, 0.0465, Rotation2d.k180deg);

    private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();

    private final NetworkTable questNavTable = nt4Instance.getTable("QuestNav");
    
    private final Pose2dProto pose2dProto = new Pose2dProto();

    private final DeviceDataProto deviceDataProto = new DeviceDataProto();

    private final FrameDataProto frameDataProto = new FrameDataProto();


    private final ProtobufSubscriber<Data.ProtobufQuestNavDeviceData> deviceDataSubscriber =
        questNavTable
        .getProtobufTopic("deviceData", deviceDataProto)
        .subscribe(Data.ProtobufQuestNavDeviceData.newInstance());

    
    private final ProtobufSubscriber<Data.ProtobufQuestNavFrameData> frameDataSubscriber =
        questNavTable
            .getProtobufTopic("frameData", frameDataProto)
            .subscribe(
                Data.ProtobufQuestNavFrameData.newInstance(),
                PubSubOption.periodic(0.01),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(20));


    

    public OculusPlus() {
        super("QuestNav/FORGE", false);
        this.nav = new QuestNav();
    }

    public OptionalInt getBattery(){
        Data.ProtobufQuestNavDeviceData latestDeviceData = deviceDataSubscriber.get();

        if (latestDeviceData != null) {
          return OptionalInt.of(latestDeviceData.getBatteryPercent());
        }
        return OptionalInt.empty();
    }

    public boolean isTracking() {
        Data.ProtobufQuestNavDeviceData latestDeviceData = deviceDataSubscriber.get();
        if (latestDeviceData != null) {
          return latestDeviceData.getCurrentlyTracking();
        }
        return false; 
    }

    public boolean isConnected() {
        return Seconds.of(Timer.getTimestamp())
            .minus(Microseconds.of(frameDataSubscriber.getLastChange()))
            .lt(Milliseconds.of(50));
    }

    public OptionalInt getFrameCount() {
        Data.ProtobufQuestNavFrameData latestFrameData = frameDataSubscriber.get();
        if (latestFrameData != null) {
          return OptionalInt.of(latestFrameData.getFrameCount());
        }
        return OptionalInt.empty();
    }

    public OptionalDouble getAppTimestamp() {
        Data.ProtobufQuestNavFrameData latestFrameData = frameDataSubscriber.get();
        if (latestFrameData != null) {
          return OptionalDouble.of(latestFrameData.getTimestamp());
        }
        return OptionalDouble.empty();
    }

    public PoseFrame[] getAllUnreadPoseFrames() {
        var frameDataArray = frameDataSubscriber.readQueue();
        var result = new PoseFrame[frameDataArray.length];
        for (int i = 0; i < result.length; i++) {
          var frameData = frameDataArray[i];
          result[i] =
              new PoseFrame(
                  pose2dProto.unpack(frameData.value.getPose2D()),
                  Microseconds.of(frameData.serverTime).in(Seconds),
                  frameData.value.getTimestamp(),
                  frameData.value.getFrameCount());
        }
        return result;
    }

    public Pose2d getPose(){
        PoseFrame[] poseFrames = getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = questPose.transformBy(robotToQuest.inverse());
            return robotPose;
        }
        return null;

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

    public boolean hasPose(){
        return nav.isConnected() && nav.isTracking();
    }




}
