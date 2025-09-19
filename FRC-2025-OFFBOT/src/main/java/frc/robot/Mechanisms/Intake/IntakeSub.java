package frc.robot.Mechanisms.Intake;

import lib.ForgePlus.Math.Profiles.Control.FeedForwardControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import frc.robot.Mechanisms.MechanismsConstants;
import frc.robot.Mechanisms.MechanismsConstants.IntakeConstants;
import frc.robot.Mechanisms.MechanismsConstants.OutConstants;

public class IntakeSub extends NetworkSubsystem{

    private ForgeSparkMax motorAng;
    private PIDControl pidUp;
    private PIDControl pidDown;

    private TalonFX wheels;
    private TalonFXConfiguration IntConfigs;

    //private SparkAbsoluteEncoder encoder;
    private RelativeEncoder relEncoder;

    public IntakeSub () {
        super("IntakeSubsystem", false);

        motorAng = new ForgeSparkMax(IntakeConstants.IntAngle_ID, "IntakeAngle");
        wheels = new TalonFX(IntakeConstants.IntWheels_ID);

        //encoder = motorAng.getAbsoluteEncoder();
        relEncoder = motorAng.getEncoder();
        pidUp = new PIDControl(IntakeConstants.pidGainsUp);
        pidDown = new PIDControl(IntakeConstants.pidGainsDown);

        //Config Motor Angulador
        motorAng.flashConfiguration(
        MechanismsConstants.IntakeConstants.AngulatorInverted,
        IdleMode.kCoast,
        MechanismsConstants.IntakeConstants.AngulatorCurrentLimit,
        false);
        
        pidUp.setTolerance(OutConstants.pidTolerance);
        pidDown.setTolerance(OutConstants.pidTolerance);
        
        //Config Motor Ruedas
        IntConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        IntConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        wheels.getConfigurator().apply(IntConfigs);
    }


    @Override
    public void NetworkPeriodic(){ }

    @AutoNetworkPublisher(key = "AngPosition")
    private double getPosition(){ 
        return (relEncoder.getPosition() / 17.9) * 360;     
    }

    public void setEncoderPosition(double position) {
        relEncoder.setPosition(position);
    }

    public void setPosition(double targetPosition) {
        double output;

        if (targetPosition > 0){
            double Up = pidUp.calculate(getPosition(), targetPosition).getOutput();
            double ff = FeedForwardControl.calculate(OutConstants.FFgains, OutConstants.ffVelocity, OutConstants.ffAceleration).getOutput();
            output = Up + ff;
        }else{
            double Down = pidDown.calculate(getPosition(), targetPosition).getOutput();
            output = Down; 
        }        

        motorAng.set(output); 
    }

    public void runIntake(double speed){
        motorAng.set(speed);
    }

    
    @AutoNetworkPublisher(key = "CurrentSetpoint")
    public double currentSetpoint(){
        return pidUp.getSetpoint()*pidDown.getSetpoint() >= 0 ? pidUp.getSetpoint() : pidDown.getSetpoint();
    }

    @AutoNetworkPublisher(key = "AtGoal")
    public boolean atGoal(){
        return  (getPosition() - currentSetpoint()) <=  OutConstants.armTolerance;
    }

    //Obtener velocidad motorPID
    public double getVelocity() {
        return relEncoder.getVelocity();
    }

    public void runWheelsIntake(double speed) {  
        wheels.set(speed);
    }

    public void stopAng() {
        motorAng.stopMotor();
    }

    public void stopWheelsIntake() {
        wheels.stopMotor();
    }

    public void stopAll(){
        motorAng.stopMotor();
        wheels.stopMotor();
    }
    
}

