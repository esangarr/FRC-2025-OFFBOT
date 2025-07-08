package frc.robot.DriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ForgePlus.Equals.Conditional;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.NetworkCommand;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.Sim.Annotations.RealDevice;
import lib.ForgePlus.Sim.SimulatedSubsystem;
import lib.ForgePlus.SwerveLib.Odometer.FieldObject;
import lib.ForgePlus.SwerveLib.PathFinding.PoseFinder;
import lib.ForgePlus.SwerveLib.Utils.SwerveModuleStateSupplier;
import lib.ForgePlus.SwerveLib.Visualizers.ElasticField;
import lib.ForgePlus.SwerveLib.Visualizers.SwerveWidget;

 public class Swerve extends NetworkSubsystem implements SimulatedSubsystem{

    @RealDevice
    private final AHRS navX;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleLocations());

    private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    
    private final PIDConstants translationPPgains; 
    private final PIDConstants rotationPPgains;
    
    private final PoseFinder pathFinder;

    private SwerveModule[] modules = new SwerveModule[4];

    public PathConstraints selectedConstraints;

    public ElasticField dashboardField;

    public OculusPlus nav = new OculusPlus();
    

    public enum SwervePathConstraints{
        kFast, kNormal
    }
    public Swerve(SwervePathConstraints constraints){
        super("FORGESwerve", false);

        initializeSubsystemDevices();
        nav.resetPose();

        this.selectedConstraints = Conditional.chooseBetween(
            SwerveConstants.normalPathConstraints,
            SwerveConstants.fastPathConstraints,
            constraints == SwervePathConstraints.kNormal);

        if (isInSimulation()) {
            navX = null;
            translationPPgains = new PIDConstants(5.0, 0,0);
            rotationPPgains = new PIDConstants(5.0, 0,0);
        }else{
            navX = new AHRS(NavXComType.kMXP_SPI);

            translationPPgains = new PIDConstants(5.5, 0, 0); 
            rotationPPgains = new PIDConstants(2.93, 0.0, 0.001);
        }

        modules[0] = new SwerveModule(0);
        modules[1] = new SwerveModule(1);
        modules[2] = new SwerveModule(2);
        modules[3] = new SwerveModule(3);
    
        if (!isInSimulation()) {
            new Thread(() -> {
                try{
                    Thread.sleep(1000);
                    navX.reset();
                    nav.resetPose();
                
                } catch (Exception e){
        
                }
              }).start();
        }

        AutoBuilder.configure(this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
      new PPHolonomicDriveController(
        new PIDConstants(5.5, 0.0,0),
         new PIDConstants(2.81, 0.0, 0.001)),
      getPathPlannerConfiguration(),
      () -> DriverStation.getAlliance().
        orElse(Alliance.Blue) == Alliance.Red,
      this);

        pathFinder = new PoseFinder(
            FieldObject.REEFSCAPE,
            selectedConstraints,
            this::runVelocity,
            this::getEstimatedPosition,
            this::setPose,
            0.02,
            this);

        SwerveModuleStateSupplier[] suppliers = new SwerveModuleStateSupplier[4];

        suppliers[0] = new SwerveModuleStateSupplier(
            ()-> modules[0].getModuleVelocity(),
            ()-> modules[0].getModuleRotation().getRadians());

        suppliers[1] = new SwerveModuleStateSupplier(
            ()-> modules[1].getModuleVelocity(),
            ()-> modules[1].getModuleRotation().getRadians());
            
        suppliers[2] = new SwerveModuleStateSupplier(
            ()-> modules[2].getModuleVelocity(),
            ()-> modules[2].getModuleRotation().getRadians());

        suppliers[3] = new SwerveModuleStateSupplier(
            ()-> modules[3].getModuleVelocity(),
            ()-> modules[3].getModuleRotation().getRadians());

        SwerveWidget.buildCustomPath(
            getTableKey(),
            "Elastic/SwerveWidget",
            suppliers[0],
            suppliers[1],
            suppliers[2],
            suppliers[3],
            ()-> getRotation().getRadians()
        );

        dashboardField = new ElasticField(
            getTableKey(), "Elastic/Field", this::getEstimatedPosition);

        publishOutput("PoseFinder", pathFinder);

        publishOutput("IsInSim", isInSimulation());

    }

    private Translation2d[] getModuleLocations(){
        return new Translation2d[] {
            new Translation2d(SwerveConstants.TRACK_WIDTH_X / 2.0, SwerveConstants.TRACK_WIDTH_Y / 2.0),
            new Translation2d(SwerveConstants.TRACK_WIDTH_X / 2.0, -SwerveConstants.TRACK_WIDTH_Y / 2.0),
            new Translation2d(-SwerveConstants.TRACK_WIDTH_X / 2.0, SwerveConstants.TRACK_WIDTH_Y / 2.0),
            new Translation2d(-SwerveConstants.TRACK_WIDTH_X / 2.0, -SwerveConstants.TRACK_WIDTH_Y / 2.0)
            };
    }

    private RobotConfig getPathPlannerConfiguration(){

      return new RobotConfig(
        SwerveConstants.ROBOTMASSKG,
        SwerveConstants.ROBOTMOI,
            new ModuleConfig(
              SwerveModule.WHEELRADIUS,
              SwerveConstants.MAX_LINEAR_SPEED,
              1.0,
              SwerveModule.NEOGearbox.
                withReduction(SwerveModule.driveMotorReduction),
              SwerveModule.driveCurrentLimit,
              1),
        getModuleLocations());
    }

    private boolean gyroConnection(){
        if (isInSimulation()) {
            return false;
        }
        return navX.isConnected();
    }

    public double getAngle(){
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public Rotation2d getnavXRotation(){
        return Rotation2d.fromDegrees(-getAngle());
    }

    
    @Override
    public void NetworkPeriodic(){

        for (var module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
              module.stopModule();
        }}

        publishOutput("Odometry/NormalPose", estimator.getEstimatedPosition());

        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }
        
        if (gyroConnection() == true) {
            rawGyroRotation = getnavXRotation();
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
        nav.update();
        
        if (nav.hasPose()) {
            estimator.addVisionMeasurement(nav.getPose(), nav.timestamp() , nav.dev.asVector());
        }
        

        estimator.update(rawGyroRotation, modulePositions);

        publishOutput("Odometry/BotPose", estimator.getEstimatedPosition());
        publishOutput("Odometry/QuestPose", nav.getPose());

    }

    

    @Override
    public void RealDevicesPeriodic(){
    }

    


    @Override
    public void SimulationDevicesPeriodic(){}

    public void runVelocity(ChassisSpeeds speeds) {
        //Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_LINEAR_SPEED);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
          modules[i].runCharacterization(output);
        }
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
          output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
      }

    public void stop(){
        runVelocity(new ChassisSpeeds());
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
      }

    public void homeModules(){
        for (int i = 0; i < 4; i++) {
              modules[i].toHome();
          }
    }

    @NetworkCommand("Commands/HomeModules")
    public Command homeModulesCommand(){
        return Commands.runOnce(()-> homeModules(), this);
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
          headings[i] = getModuleLocations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void resetHeading(){
        setPose(
            new Pose2d(
            getEstimatedPosition().getX(),
            getEstimatedPosition().getY(),
            new Rotation2d())
        );
    }

    @AutoNetworkPublisher(key = "Modules/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
        }
        return states;
    }

    @AutoNetworkPublisher(key = "Modules/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
        }
        return states;
    }

    @AutoNetworkPublisher(key = "Modules/ChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds(){
      return kinematics.toChassisSpeeds(getModuleStates());
    } 


    public void setPose(Pose2d pose) {
        nav.setPoseQuest(pose);
        estimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        
    }

    @AutoNetworkPublisher(key = "Odometry/BotPose2D")
    public Pose2d getEstimatedPosition() {
        return estimator.getEstimatedPosition();
    }

    @AutoNetworkPublisher(key = "Odometry/BotHeading")    
    public Rotation2d getRotation(){
        return getEstimatedPosition().getRotation();
    }

    public PoseFinder getPathFinder(){
        return pathFinder;
    }

    @AutoNetworkPublisher(key = "Speeds/MaxLinear") 
    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return SwerveConstants.MAX_LINEAR_SPEED;
    }

    @AutoNetworkPublisher(key = "Speeds/MaxAngular")      
    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return SwerveConstants.MAX_ANGULAR_SPEED;
    }
}