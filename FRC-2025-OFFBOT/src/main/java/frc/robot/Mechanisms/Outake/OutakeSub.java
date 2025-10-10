
package frc.robot.Mechanisms.Outake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Mechanisms.MechanismsConstants.OutConstants;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

public class OutakeSub extends NetworkSubsystem{

    private ForgeSparkMax arm; 
    private PIDControl pidUp;
    private PIDControl pidDown;

    private TalonFX wheels;
    private TalonFXConfiguration wheelsConfig;

    private RelativeEncoder relEncoder ;


    public OutakeSub () {
        super("OutakeSubsystem", false);

        arm = new ForgeSparkMax(OutConstants.arm_ID, "OutakeAngle");
        pidUp = new PIDControl(OutConstants.pidGainsUp);
        pidDown = new PIDControl(OutConstants.pidGainsDown);

        wheels = new TalonFX(OutConstants.Wheels_ID);
        wheels.setNeutralMode(NeutralModeValue.Brake);

        wheelsConfig = new TalonFXConfiguration();

        wheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        wheelsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        relEncoder = arm.getEncoder();

        arm.flashConfiguration(
            OutConstants.armInveerted, 
            IdleMode.kBrake, 
            OutConstants.armCurrentLimit, 
            false);


        pidUp.setTolerance(OutConstants.pidTolerance);
        pidDown.setTolerance(OutConstants.pidTolerance);
    }

    @Override
    public void NetworkPeriodic(){
        publish("angleoutake", getPosition());
        publish("CurrentSetpoint", currentSetpoint());
        publish("GetCurrent", getCurrent());
    }

    public double getPosition(){ 
        return (relEncoder.getPosition() / 66.666) * 360;
    }

    public double getCurrent(){
        return wheels.getTorqueCurrent().getValueAsDouble();
    }

    
    public void resetEconder(){
        relEncoder.setPosition(0);
    }

    public void setPositionUp(double targetPosition) {

        double output;

        output = pidUp.calculate(targetPosition, getPosition()).getOutput();

        arm.set(output);

    }

    public void setPositionDown(double targetPosition) {

        double output;

        output = pidDown.calculate(targetPosition, getPosition()).getOutput();

        arm.set(output);

    }


    public void runArm(double speed){
        arm.set(speed);
    }

    public void runWheelsOutake(double speed){
        wheels.set(speed);
    }

    public double currentSetpoint(){
        return pidUp.getSetpoint()*pidDown.getSetpoint() >= 0 ? pidUp.getSetpoint() : pidDown.getSetpoint();
    }

    public boolean isWheelSpinning(){
        return wheels.get() != 0;
    }

    public boolean atGoal(){
        return pidDown.atSetpoint() || pidUp.atSetpoint();
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
    

