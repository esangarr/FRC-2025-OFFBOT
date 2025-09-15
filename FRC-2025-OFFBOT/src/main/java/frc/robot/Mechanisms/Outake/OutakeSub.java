package frc.robot.Mechanisms.Outake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Mechanisms.MechanismsConstants.IndexerConstants;
import frc.robot.Mechanisms.MechanismsConstants.OutConstants;
import lib.ForgePlus.Math.Profiles.Control.ControlResult;
import lib.ForgePlus.Math.Profiles.Control.FeedForwardControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

public class OutakeSub extends NetworkSubsystem{

    private ForgeSparkMax arm; 
    private PIDControl pid;

    private TalonFX wheels;
    private TalonFXConfiguration wheelsConfig;

    private SparkAbsoluteEncoder encoder;


    public OutakeSub () {
        super("OutakeSubsystem", false);

        arm = new ForgeSparkMax(OutConstants.arm_ID,"OutakeAngle");
        pid = new PIDControl(OutConstants.pidGains);

        wheels = new TalonFX(OutConstants.Wheels_ID);
        wheels.setNeutralMode(NeutralModeValue.Brake);

        wheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        wheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        encoder = arm.getAbsoluteEncoder();

        arm.flashConfiguration(
            OutConstants.armInveerted, 
            IdleMode.kCoast, 
            OutConstants.armCurrentLimit, 
            false);


        pid.setTolerance(OutConstants.pidTolerance);
    }

    @Override
    public void NetworkPeriodic(){}

    @AutoNetworkPublisher(key = "Position")
    private double getPosition(){ //Request Position
        return encoder.getPosition() * 360; 
    }

    private void setPosition(double targetPosition) {
        double ff = FeedForwardControl.calculate(OutConstants.FFgains, OutConstants.ffVelocity, OutConstants.ffAceleration).getOutput();
        double PID = pid.calculate(getPosition(), targetPosition).getOutput();

        double output = PID + ff;
        //output = Math.max(OutConstants.Wrist_MinOutput, Math.min(OutConstants.Wrist_MaxOutput, output));//clamp
        arm.set(output); 
    }

    private void runArm(double speed){
        arm.set(speed);
    }

    private void runWheels(double speed){
        wheels.set(speed);
    }

    @AutoNetworkPublisher(key = "CurrentSetpoint")
    private double currentSetpoint(){
        return pid.getSetpoint();
    }

    @AutoNetworkPublisher(key = "AtGoal")
    private boolean atGoal(){
        return (getPosition() - currentSetpoint()) <=  OutConstants.armTolerance;
    }

    private void stopArm(){
        arm.stopMotor();
    }

    private void stopwheels(){
       wheels.stopMotor();
    }

    private void stopALL(){
       wheels.stopMotor();
        arm.stopMotor();
    }
}
    

