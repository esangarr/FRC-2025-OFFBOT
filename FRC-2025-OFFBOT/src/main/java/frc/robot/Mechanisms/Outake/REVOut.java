package frc.robot.Mechanisms.Outake;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Mechanisms.Outake.OutConstants;

import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;


public class REVOut extends SubsystemBase{
    private PIDController wristPID;
    private ForgeSparkMax CANWrist;   
    //private SparkMaxConfig WristConfig;  flashConfiguration kinda does it

public REVOut(){
        CANWrist = new ForgeSparkMax(OutConstants.CANWrist_ID);
        wristPID = new PIDController(OutConstants.CANWrist_kP, OutConstants.CANWrist_kI, OutConstants.CANWrist_kD);
        wristPID.setTolerance(OutConstants.WristTolerance);
        Burnflash();
    }

private void Burnflash(){
        CANWrist.setCANTimeout(250);
        CANWrist.flashConfiguration(
            false, 
            IdleMode.kBrake,
            25,
            false);
            //positionConversionFactor is worth looking into
    }
    private double getCurrentPosition(){ //Request Position
        return CANWrist.getAlternateEncoder().getPosition() * 360; 
        //ForgeSparkMax's .getPosition().getRead(); might work because the config uses an AlternatEnc object
    }

    private void setPosition(double targetPosition) {
        double output = wristPID.calculate(getCurrentPosition(), targetPosition);
        output = Math.max(
            OutConstants.Wrist_MinOutput,     Math.min(OutConstants.Wrist_MaxOutput, output));//clamp
        CANWrist.set(output); 
}

/* matate stv - JM
public double FeedForwardCalculate(double kA, double kV, double kG, double kS){
    ArmFeedforward feedforward = new ArmFeedforward(kS, kV, kG, kA);
        return feedforward.calculate(1, 2, 3);
} //Esta es la ofrma en la que me marcaba wplib, no c si este bien, si si, alch que tibios somos

//La que sigue es la fumada que se sacaron en leon
/public double FeedForwardCalculate(double kA, double kV, double kG, double kS, double velocity, double acceleration, double angle){
   
double feedforwardout = kS * Math.signum(velocity) + kV * velocity + kG + kA * acceleration;
    return feedforwardout;
}   
*/

/*Methods go here:
    TODO: Setpoint
zeroOffset is worth looking into
*/

private boolean atGoal(){
    return wristPID.atSetpoint();
}

private void stopWrist(){
    CANWrist.stopMotor();
}
}