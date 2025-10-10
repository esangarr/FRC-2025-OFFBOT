package frc.robot.Mechanisms.Intake;

import lib.ForgePlus.Math.Profiles.Control.FeedForwardControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private FeedForwardControl ff;

    private TalonFX wheels;
    private TalonFXConfiguration IntConfigs;

    private SparkAbsoluteEncoder encoder;

    public IntakeSub () {
        super("IntakeSubsystem", false);

        motorAng = new ForgeSparkMax(IntakeConstants.IntAngle_ID, "IntakeAngle");
        wheels = new TalonFX(IntakeConstants.IntWheels_ID);

        encoder = motorAng.getAbsoluteEncoder();
        pidUp = new PIDControl(IntakeConstants.pidGainsUp);
        pidDown = new PIDControl(IntakeConstants.pidGainsDown);

        ff = new FeedForwardControl();


        //Config Motor Angulador
        motorAng.flashConfiguration(
        MechanismsConstants.IntakeConstants.AngulatorInverted,
        IdleMode.kBrake,
        MechanismsConstants.IntakeConstants.AngulatorCurrentLimit,
        true);
        
        pidUp.setTolerance(IntakeConstants.pidTolerance);
        pidDown.setTolerance(IntakeConstants.pidTolerance);

        //Config Motor Ruedas
        IntConfigs = new TalonFXConfiguration();

        IntConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        IntConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        wheels.getConfigurator().apply(IntConfigs);
    }


    @Override
    public void NetworkPeriodic(){
        publishOutput("Angulo", getPositionAng());
        publishOutput("CurrentSetpoint", currentSetpoint());
        publishOutput("Atgoal",  atGoal());
    }

    public double getPositionAng(){ 
        return (encoder.getPosition()  * 360); 

    }

    public void setVoltage(double voltage){
        motorAng.setVoltage(voltage);
    }


    public void setPositionUp(double targetPosition) {

        double output;

        double pid = pidUp.calculate(targetPosition, getPositionAng()).getOutput();
        double feedForward = FeedForwardControl.calculate(IntakeConstants.FFgains, 0 , 0).getOutput();

        output = pid + feedForward;

        motorAng.set(output);

    }

    public void setPositionDown(double targetPosition) {

        double output;

        output = pidDown.calculate(targetPosition, getPositionAng()).getOutput();

        motorAng.set(output);

    }

    public void runIntake(double speed){
        motorAng.set(speed);
    }

    
    public double currentSetpoint(){
        return pidUp.getSetpoint()*pidDown.getSetpoint() >= 0 ? pidUp.getSetpoint() : pidDown.getSetpoint();
    }

    public boolean atGoal(){
        return Math.abs(getPositionAng() - currentSetpoint()) <=  IntakeConstants.intakeTolerance;
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

