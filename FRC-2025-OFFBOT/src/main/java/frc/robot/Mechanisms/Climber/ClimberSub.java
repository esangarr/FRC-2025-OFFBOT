package frc.robot.Mechanisms.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Mechanisms.MechanismsConstants.ClimberConstants;
import frc.robot.Mechanisms.MechanismsConstants.ElevatorConstants;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

public class ClimberSub extends NetworkSubsystem{

    private ForgeSparkMax climberWheels;

    private TalonFX climber;
    private TalonFXConfiguration climbConfig;
    

    public ClimberSub () {
        super("ClimberSubsystem", false);

        climberWheels = new ForgeSparkMax(ClimberConstants.ClimbWheel_ID
        
        , MotorType.kBrushless, "MotorClimb");

        climberWheels.flashConfiguration(
            ClimberConstants.cimberInverted, 
            IdleMode.kBrake, 
            0, 
            false);


        climber = new TalonFX(ClimberConstants.Climber_ID);
        climber.setNeutralMode(NeutralModeValue.Brake);

        climbConfig = new TalonFXConfiguration();

        climbConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    }

    @Override
    public void NetworkPeriodic(){ }

    public void AngulateClimber(double speed){
        climber.set(speed);
    }

    public void runWheels(double speed){
        climberWheels.set(speed);
    }

    public void StopAng(){
        climber.stopMotor();
    }

    public void StopWheels(){
        climberWheels.stopMotor();
    }

    public void StopAll(){
        climber.stopMotor();
        climberWheels.stopMotor();
    }
    
}
