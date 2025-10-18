package frc.robot.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.   Math.Profiles.ProfileGains.PIDGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.SimpleFeedForwardGains;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

public class SwerveModule{

    public static final int driveCurrentLimit = 40;

    public static final int turnCurrentLimit = 20;
    public static final DCMotor NEOGearbox = DCMotor.getNEO(1);
    public static final double WHEELRADIUS = Units.inchesToMeters(2.0);
    public static final double driveMotorReduction = 5.36;
    public static final double turnMotorReduction =  18.75;

    private ForgeSparkMax driveSparkMax;
  
    private ForgeSparkMax turnSparkMax;
  
    private CANcoder absoluteEncoder;

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
     
            this.turnPIDGains = new PIDGains(2.5, 0,0.0001);

            this.drivePIDGains = new PIDGains(1.25, 0, 0.001);
            this.driveFFGains = new SimpleFeedForwardGains(0, 0, 0);
            
            createSparks(index);


        drivePID = new PIDControl(drivePIDGains);
        turnPID = new PIDControl(turnPIDGains);

        turnPID.continuousInput(-Math.PI, Math.PI);


    }

    //Main Loop
    public void periodic(){
        RealDevicesPeriodic();

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

        SmartDashboard.putNumber("Vel", driveSparkMax.getVelocity().getRead());
    }

    public void RealDevicesPeriodic(){

        this.moduleAngle = 
        absoluteEncoder.isConnected() ? 
            Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble() - Offset) : 
            Rotation2d.fromRotations(turnSparkMax.getPosition().withReduction(turnMotorReduction).getRead());

        this.driveVelocity = driveSparkMax.getVelocity().toRadiansPerSecond().getRead() / driveMotorReduction * WHEELRADIUS;

        SmartDashboard.putNumber("velfux", driveVelocity);

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
        return driveVelocity;
    }

    public double getFFCharacterizationVelocity() {
        return driveVelocity;
      }

    public double getDrivePositionMeters(){

        double position = driveSparkMax.getPosition().toRadians().getRead() / driveMotorReduction;

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
        setDriveVelocity(speedSetpoint/ WHEELRADIUS);

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
            60,
            true);
        
        turnSparkMax.flashConfiguration(
            isTurnMotorInverted,
            IdleMode.kBrake,
            30,
            true);

    }
}