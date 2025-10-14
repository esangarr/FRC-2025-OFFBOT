
package frc.robot.Mechanisms.Outake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.MechanismsConstants.OutConstants;
import lib.ForgePlus.Math.Profiles.Control.FeedForwardControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

public class OutakeSub extends NetworkSubsystem{

    private TalonFX arm;


    private TalonFX wheels;
    private TalonFXConfiguration wheelsConfig;

    private TalonFXConfiguration armConfig;
    
    private PositionVoltage arm_request;

    private double rotorPosLatency;
    private double rotorPosRotations;



    public OutakeSub () {
        super("OutakeSubsystem", false);

        arm = new TalonFX(OutConstants.arm_ID);
        armConfig = new TalonFXConfiguration();

        arm_request = new PositionVoltage(0);

        wheels = new TalonFX(OutConstants.Wheels_ID);

        wheelsConfig = new TalonFXConfiguration();

        wheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        wheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        wheels.getConfigurator().apply(wheelsConfig);
        arm.getConfigurator().apply(armConfig);

        configMotion();

    
    }

    @Override
    public void NetworkPeriodic(){

        updatePosition();
    
        SmartDashboard.putNumber("angleoutake", getPosition());
        SmartDashboard.putNumber("Rotations", rotorPosRotations);
        //SmartDashboard.putNumber("CurrentSetpoint", currentSetpoint());
        SmartDashboard.putNumber("GetOutSetpoint", getSetpoint());

        SmartDashboard.putBoolean("Is Spinning", isWheelSpinning());


    }

    public void updatePosition(){
        // acquire a refreshed TalonFX rotor position signal
        var rotorPosSignal = arm.getRotorPosition();

        rotorPosRotations = rotorPosSignal.getValueAsDouble();

        rotorPosLatency = rotorPosSignal.getTimestamp().getLatency();

    }

    public void configMotion(){
        
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = 0.35; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 1.3; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0 ; // A velocity error of 1 rps results in 0.1 V output
    

        var slot1Configs = talonFXConfigs.Slot1;

        slot1Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot1Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0 ; // A velocity error of 1 rps results in 0.1 V output
    
        
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        arm.getConfigurator().apply(talonFXConfigs);

    }

    public enum OutakeRequestType{
        kUp,
        KDown,
        KAlgae
    }

    public void setPosition(double position, OutakeRequestType type){
        if (type == OutakeRequestType.kUp){
        arm.setControl(arm_request.withPosition(position).withSlot(0));}
        else if (type == OutakeRequestType.KDown){
        arm.setControl(arm_request.withPosition(position).withSlot(1));
        }
        

    }

    public double getSetpoint(){
        return RotationsToDegrees(arm.getClosedLoopReference().getValueAsDouble());
    }


    public double getPosition(){ 
        return (rotorPosRotations / 66.666) * 360;
    }

    public double RotationsToDegrees(double rotations){
        return (rotations / 66.666) * 360;
    }

    public double DegreesToRotations(double degrees){
        double gamma = degrees / 360;
        return gamma * 66.666;
    }

    public double getCurrent(){
        return wheels.getTorqueCurrent().getValueAsDouble();
    }

     

    public void setVoltage(double voltage){
        arm.setVoltage(voltage);
    }


    public void runArm(double speed){
        arm.set(speed);
    }

    public void runWheelsOutake(double speed){
        wheels.set(speed);
    }


    public boolean isWheelSpinning(){
        return wheels.get() != 0;
    }

    public void stopArm(){
        arm.stopMotor();
    }

    public void stopwheelsOutake(){
       wheels.stopMotor();
    }

    public void stopALL(){
       wheels.stopMotor();
        arm.stopMotor();
    }
}
    

