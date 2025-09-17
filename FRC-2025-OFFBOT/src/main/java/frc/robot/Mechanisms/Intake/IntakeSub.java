package frc.robot.Mechanisms.Intake;

import lib.ForgePlus.Math.Profiles.Control.FeedForwardControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.function.BooleanConsumer;
import frc.robot.Mechanisms.MechanismsConstants;
import frc.robot.Mechanisms.MechanismsConstants.IntakeConstants;
import frc.robot.Mechanisms.MechanismsConstants.OutConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends NetworkSubsystem{

    private ForgeSparkMax motorAng;
    private PIDControl pidUp;
    private PIDControl pidDown;

    private TalonFX wheels;
    private TalonFXConfiguration IntConfigs;

    private SparkAbsoluteEncoder encoder;

    public IntakeSub () {
        super("IntakeSubsystem", false);

        // Motor Angulador
        motorAng = new ForgeSparkMax(IntakeConstants.IntAngle_ID, "IntakeAngle");
        encoder = motorAng.getAbsoluteEncoder();
        pidUp = new PIDControl(IntakeConstants.pidGainsUp);
        pidDown = new PIDControl(IntakeConstants.pidGainsDown);

        //Config Motor Angulador

        motorAng.flashConfiguration(
        MechanismsConstants.IntakeConstants.Angulator,
        IdleMode.kCoast,
        MechanismsConstants.IntakeConstants.AngulatorCurrentLimit,
        false);
        
        pidUp.setTolerance(OutConstants.pidTolerance);
        pidDown.setTolerance(OutConstants.pidTolerance);

        //Motor Wheels
        wheels = new TalonFX(IntakeConstants.IntWheels_ID);
            

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
        return encoder.getPosition() * 360;     
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

        //output = Math.max(OutConstants.Wrist_MinOutput, Math.min(OutConstants.Wrist_MaxOutput, output));//clamp
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
        return encoder.getVelocity();
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

