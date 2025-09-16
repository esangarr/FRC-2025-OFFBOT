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
import frc.robot.Mechanisms.MechanismsConstants;
import frc.robot.Mechanisms.MechanismsConstants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends NetworkSubsystem{

    private final ForgeSparkMax motorAng;
    private final PIDControl pid;

    private TalonFX KrakenIntake;
    private TalonFXConfiguration IntConfigs;

    private final SparkAbsoluteEncoder encoder;

    public IntakeSub () {
        super("IntakeSubsystem", false);

              // Motor Angulador
              motorAng = new ForgeSparkMax(IntakeConstants.IntAngle_ID, "IntakeAngle");
              encoder = motorAng.getAbsoluteEncoder();
              pid = new PIDControl(IntakeConstants.pidGains);

              //Config Motor Angulador

              motorAng.flashConfiguration(
                MechanismsConstants.IntakeConstants.Angulator,
                IdleMode.kCoast,
                MechanismsConstants.IntakeConstants.AngulatorCurrentLimit,
                false);
        
                pid.setTolerance(MechanismsConstants.IntakeConstants.pidTolerance);

                //Motor Wheels
                KrakenIntake = new TalonFX(IntakeConstants.IntWheels_ID);
            

                //Config Motor Ruedas

                IntConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
                IntConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                IntConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                KrakenIntake.getConfigurator().apply(IntConfigs);
    }


    @Override
    public void NetworkPeriodic(){ 

    }

     @AutoNetworkPublisher(key = "AngPosition")
    private double getPosition(){ //Request Position
        return encoder.getPosition() * 360;     
    }

    private void setPosition(double targetPosition) {

        double ff = FeedForwardControl.calculate(
            IntakeConstants.FFgains, 
            IntakeConstants.ffVelocity, 
            IntakeConstants.ffAceleration).getOutput();

        double PID = pid.calculate(getPosition(), targetPosition).getOutput();

        double output = PID + ff;
        //output = Math.max(OutConstants.Wrist_MinOutput, Math.min(OutConstants.Wrist_MaxOutput, output));//clamp
        motorAng.set(output); 
    }

    private void setVel (double speed){

        motorAng.set(speed);
    }

    @AutoNetworkPublisher(key = "AngSetpoint")
    private double currentSetpoint(){
        return pid.getSetpoint();
    }

    //Obtener velocidad motorPID
    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void setvel(double speed) {  
        KrakenIntake.set(speed);
    }

    public void stopAng() {
        motorAng.stopMotor();
        
    }

    public void stopWheels() {
        KrakenIntake.stopMotor();
        
    }

    public void stopAllInt() {
        motorAng.stopMotor();
        KrakenIntake.stopMotor();
        
    }
    
}

