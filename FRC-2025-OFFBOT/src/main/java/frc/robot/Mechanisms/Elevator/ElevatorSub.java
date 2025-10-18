
package frc.robot.Mechanisms.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.MechanismsConstants.ElevatorConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class ElevatorSub extends NetworkSubsystem{

    private TalonFX leader, follower;

    private TalonFXConfiguration LeaderConfig, FollowerConfig;
    private TalonFXConfigurator leaderConfigurator, followerConfigurator;


    private  MotionMagicExpoVoltage leader_request;

    private double rotorPosLatency;
    private double rotorPosRotations;


    public ElevatorSub () {
        super("ElevatorSubsystem", false);

        leader = new TalonFX(ElevatorConstants.Leader_ID);
        follower = new TalonFX(ElevatorConstants.Follower_ID);

        leader_request = new MotionMagicExpoVoltage(0);

        LeaderConfig = new TalonFXConfiguration();
        leaderConfigurator = leader.getConfigurator();

        FollowerConfig = new TalonFXConfiguration();
        followerConfigurator = follower.getConfigurator();

        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;



        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 80;
        limitConfigs.StatorCurrentLimitEnable = true;


        follower.setControl(new Follower(leader.getDeviceID(), true));

    
        leader.getConfigurator().apply(limitConfigs);   
        
        follower.getConfigurator().apply(limitConfigs);

        leaderConfigurator.refresh(motorConfigs);
        leaderConfigurator.apply(motorConfigs);

        followerConfigurator.refresh(motorConfigs);
        followerConfigurator.apply(motorConfigs);



        configMotion();

    }

    public void updatePosition(){
        // acquire a refreshed TalonFX rotor position signal
        var rotorPosSignal = leader.getRotorPosition();

        rotorPosRotations = rotorPosSignal.getValueAsDouble();

        rotorPosLatency = rotorPosSignal.getTimestamp().getLatency();


    }

    public void configMotion(){
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = 0.33; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.1428; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.014; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.07; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.001 ; // A velocity error of 1 rps results in 0.1 V output
    
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 42; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 84;

        var slot1Configs = talonFXConfigs.Slot1;

        slot1Configs.kS = 0.20; // Add 0.25 V output to overcome static friction
        slot1Configs.kV = 0.127; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kA = 0.014; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = 0.07; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0.001 ; // A velocity error of 1 rps results in 0.1 V output
    
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 60;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        leader.getConfigurator().apply(limitConfigs);
        follower.getConfigurator().apply(limitConfigs);

        leader.getConfigurator().apply(talonFXConfigs);
        follower.getConfigurator().apply(talonFXConfigs);

        
    }

    public enum RequestType{
        kUP, kDown
    }

    public void setPosition(double position, RequestType type){

        if(type == RequestType.kUP){
            leader.setControl(leader_request.withPosition(position).withSlot(0));
        }else{
            leader.setControl(leader_request.withPosition(position).withSlot(1));
        }

    }


    public double getSetpoint(){
        return RotationsToMeters(leader.getClosedLoopReference().getValueAsDouble());
    }

    public boolean atGoal(){
        return Math.abs(getMeters() - getSetpoint()) <=  4;
    }


    public double getMeters(){
        double gamma = ((124.5/42));
        return gamma * -rotorPosRotations + 63.5;
    }

    public double RotationsToMeters(double rotations){
        double gamma = ((124.5/42));
        return gamma * -rotations + 63.5;
    }

    public double metersToRot(double meters){
        double gamma = ((124.5/42));
        double omega = 63.5;

        double convertion = (meters - omega) / gamma;

        return convertion;
    }

    public void setVoltage(double voltage){
        leader.setVoltage(voltage);
    }

    public void runMot(double speed){
        leader.set(speed);
    }

    public void StopMotors (){
        leader.stopMotor();
    }

    @Override
    public void NetworkPeriodic() {
        updatePosition();

    /* 
       publish("Rotor/Rotations", rotorPosRotations);
       publish("Rotor/Altura cm", getMeters());
       publish("Rotor/inverse", metersToRot(getMeters()));
       publish("Rotor/ElevatorSetpoint", getSetpoint());
       publish("Rotor/ElevatorAtGoal", atGoal());*/

       SmartDashboard.putNumber("Rotor/Rotations", rotorPosRotations);
       SmartDashboard.putNumber("Rotor/Altura cm", getMeters());
       SmartDashboard.putNumber("Rotor/inverse", metersToRot(getMeters()));
       SmartDashboard.putNumber("Rotor/ElevatorSetpoint", getSetpoint());
       SmartDashboard.putBoolean("Rotor/ElevatorAtGoal", atGoal());

       

    }
    }
    

