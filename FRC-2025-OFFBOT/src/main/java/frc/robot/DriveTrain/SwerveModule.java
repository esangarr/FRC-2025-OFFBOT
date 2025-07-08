package frc.robot.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.Math.Profiles.ProfileGains.PIDGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.SimpleFeedForwardGains;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;
import lib.ForgePlus.Sim.SimulatedSubsystem;
import lib.ForgePlus.Sim.Annotations.RealDevice;
import lib.ForgePlus.Sim.Annotations.SimulatedDevice;

public class SwerveModule implements SimulatedSubsystem{

    public static final int driveCurrentLimit = 50;
    public static final int turnCurrentLimit = 20;
    public static final DCMotor NEOGearbox = DCMotor.getNEO(1);
    public static final double WHEELRADIUS = Units.inchesToMeters(2.0);
    public static final double driveMotorReduction = 5.36;
    public static final double turnMotorReduction =  18.75;

    @RealDevice
    private ForgeSparkMax driveSparkMax;
    @RealDevice
    private ForgeSparkMax turnSparkMax;
    @RealDevice
    private CANcoder absoluteEncoder;

    @SimulatedDevice
    private DCMotorSim driveSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(NEOGearbox, 0.025, driveMotorReduction),
        NEOGearbox);
    
    @SimulatedDevice
    private DCMotorSim turnSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(NEOGearbox, 0.004, turnMotorReduction),
        NEOGearbox);

    private final PIDControl drivePID;
    private final PIDControl turnPID;

    private final SimpleFeedForwardGains driveFFGains;
    private final PIDGains drivePIDGains;
    private final PIDGains turnPIDGains;

    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;

    private double ffVolts = 0;

    private double driveVelocity;

    private double driveVoltage;
    private double turnVoltage;

    private double Offset;

    private boolean isDriveMotorInverted;
    private boolean isTurnMotorInverted;

    private Rotation2d moduleAngle = new Rotation2d();
    
    public SwerveModule(int index){
   
        //Use real Configuration
        if (!isInSimulation()) {
            this.turnPIDGains = new PIDGains(5.0, 0,0);
            this.drivePIDGains = new PIDGains(0.05, 0, 0);
            this.driveFFGains = new SimpleFeedForwardGains(0.1, 0.08, 0);
            
            createSparks(index);

        }else{
        //Use sim Configuration
            this.turnPIDGains = new PIDGains(8.0, 0,0);
            this.drivePIDGains = new PIDGains(0.05, 0, 0);
            this.driveFFGains = new SimpleFeedForwardGains(0, 0.0789, 0);

            
        }

        drivePID = new PIDControl(drivePIDGains);
        turnPID = new PIDControl(turnPIDGains);

        turnPID.continuousInput(-Math.PI, Math.PI);

        initializeSubsystemDevices();

    }

    //Main Loop
    public void periodic(){
        handleSubsystemRealityLoop();

        if (angleSetpoint != null) {
            this.turnVoltage = turnPID.calculate(moduleAngle.getRadians()).getOutput();

            if (speedSetpoint != null) {
                this.driveVoltage = drivePID.calculate(driveVelocity).plus(()-> ffVolts).getOutput();
            }else{
                drivePID.reset();
            }
        }else{
            turnPID.reset();
        }
    }

    @Override
    public void SimulationDevicesPeriodic(){

        this.moduleAngle = new Rotation2d(turnSim.getAngularPositionRad());
        this.driveVelocity = driveSim.getAngularVelocityRadPerSec();

        driveSim.setInputVoltage(MathUtil.clamp(driveVoltage, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnVoltage, -12.0, 12.0));

        driveSim.update(0.02);
        turnSim.update(0.02);
    }

    @Override
    public void RealDevicesPeriodic(){

        this.moduleAngle = 
        absoluteEncoder.isConnected() ? 
            Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble() - Offset) : 
            Rotation2d.fromRotations(turnSparkMax.getPosition().withReduction(turnMotorReduction).getRead());

        this.driveVelocity = driveSparkMax.getVelocity().toRadiansPerSecond().getRead() / driveMotorReduction * WHEELRADIUS;

        driveSparkMax.setVoltage(driveVoltage);
        turnSparkMax.setVoltage(turnVoltage);

    }

    public void setDriveVelocity(double velocity){
        this.ffVolts = driveFFGains.kS() * Math.signum(velocity) + driveFFGains.kV() * velocity;
        drivePID.setSetpoint(velocity);
    }

    public void setTurnPos(Rotation2d rot){
        turnPID.setSetpoint(rot.getRadians());
    }

    public void setDriveOpenLoop(double voltage){
        this.speedSetpoint = null;
        this.driveVoltage = voltage;
    }

    public void setTurnOpenLoop(double voltage){
        this.angleSetpoint = null;
        this.turnVoltage = voltage;
    }

    public double getDriveModuleVoltage(){
        return driveVoltage;
    }

    public double getTurnModuleVoltage(){
        return turnVoltage;
    }

    public double getModuleVelocity(){
        return driveVelocity * WHEELRADIUS;
    }

    public double getFFCharacterizationVelocity() {
        return driveVelocity;
      }

    public double getDrivePositionMeters(){

        double position = isInSimulation() ? driveSim.getAngularPositionRad() : driveSparkMax.getPosition().toRadians().getRead() / driveMotorReduction;

        return position * WHEELRADIUS;
      }

    public Rotation2d getModuleRotation(){
        return moduleAngle;
    }

    public void runSetpoint(SwerveModuleState desiredState){
        desiredState.optimize(getModuleRotation());

        desiredState.cosineScale(getModuleRotation());

        angleSetpoint = desiredState.angle;
        speedSetpoint = desiredState.speedMetersPerSecond;

        setTurnPos(angleSetpoint);
        setDriveVelocity(speedSetpoint / WHEELRADIUS);

    }

    public void runCharacterization(double output){
        setDriveOpenLoop(output);
        setTurnPos(new Rotation2d(0));
    }

    public void toHome(){
        runSetpoint(new SwerveModuleState(0, new Rotation2d()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePositionMeters(), getModuleRotation());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getModuleVelocity(), getModuleRotation());
    }

    public void stopModule(){   
        setTurnOpenLoop(0);
        setDriveOpenLoop(0);
    }

    private void createSparks(int index){
        switch (index) {
            case 0:
              driveSparkMax = new ForgeSparkMax(SwerveConstants.frontLeft.DrivePort, "FL Drive");
              turnSparkMax = new ForgeSparkMax(SwerveConstants.frontLeft.TurnPort, "FL Turn");
              absoluteEncoder = new CANcoder(SwerveConstants.frontLeft.EncPort);
              isDriveMotorInverted = SwerveConstants.frontLeft.DrivemotorReversed;
              isTurnMotorInverted = SwerveConstants.frontLeft.TurnmotorReversed;
              Offset = SwerveConstants.frontLeft.offset;

              break;
            case 1:
                driveSparkMax = new ForgeSparkMax(SwerveConstants.frontRight.DrivePort, "FR Drive");
                turnSparkMax = new ForgeSparkMax(SwerveConstants.frontRight.TurnPort, "FR Turn");
                absoluteEncoder = new CANcoder(SwerveConstants.frontRight.EncPort);
                isDriveMotorInverted = SwerveConstants.frontRight.DrivemotorReversed;
                isTurnMotorInverted = SwerveConstants.frontRight.TurnmotorReversed;
                Offset = SwerveConstants.frontRight.offset;

              break;
            case 2:
                driveSparkMax = new ForgeSparkMax(SwerveConstants.backLeft.DrivePort, "BL Drive");
                turnSparkMax = new ForgeSparkMax(SwerveConstants.backLeft.TurnPort, "BL Turn");
                absoluteEncoder = new CANcoder(SwerveConstants.backLeft.EncPort);
                isDriveMotorInverted = SwerveConstants.backLeft.DrivemotorReversed;
                isTurnMotorInverted = SwerveConstants.backLeft.TurnmotorReversed;
                Offset = SwerveConstants.backLeft.offset;
              
              break;
            case 3:
                driveSparkMax = new ForgeSparkMax(SwerveConstants.backRight.DrivePort, "BR Drive");
                turnSparkMax = new ForgeSparkMax(SwerveConstants.backRight.TurnPort, "BR Turn");
                absoluteEncoder = new CANcoder(SwerveConstants.backRight.EncPort);
                isDriveMotorInverted = SwerveConstants.backRight.DrivemotorReversed;
                isTurnMotorInverted = SwerveConstants.backRight.TurnmotorReversed;
                Offset = SwerveConstants.backRight.offset;

              break;
            default:
              throw new RuntimeException("Invalid module index");
          }

        driveSparkMax.flashConfiguration(
            isDriveMotorInverted,
            IdleMode.kBrake,
            43,
            true);
        
        turnSparkMax.flashConfiguration(
            isTurnMotorInverted,
            IdleMode.kBrake,
            20,
            true);

    }
}