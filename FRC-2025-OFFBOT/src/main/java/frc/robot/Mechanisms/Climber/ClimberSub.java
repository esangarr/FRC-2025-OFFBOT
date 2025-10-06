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

    private ForgeSparkMax climber; 

    private TalonFX climbWheels;
    private TalonFXConfiguration climbWheelsConfig;
    

    public ClimberSub () {
        super("ClimberSubsystem", false);

        climber = new ForgeSparkMax(ClimberConstants.Climber_ID, MotorType.kBrushless, "MotorClimb");

        climber.flashConfiguration(
            ClimberConstants.cimberInverted, 
            IdleMode.kBrake, 
            0, 
            false);

        climbWheels = new TalonFX(ClimberConstants.ClimbWheel_ID);
        climbWheels.setNeutralMode(NeutralModeValue.Brake);

        climbWheelsConfig = new TalonFXConfiguration();

        climbWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        climbWheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbWheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    }

    @Override
    public void NetworkPeriodic(){ }

    public void AngulateClimber(double speed){
        climber.set(speed);
    }

    public void runWheels(double speed){
        climbWheels.set(speed);
    }

    public void StopAng(){
        climber.stopMotor();
    }

    public void StopWheels(){
        climbWheels.stopMotor();
    }

    public void StopAll(){
        climber.stopMotor();
        climbWheels.stopMotor();
    }
    
}
