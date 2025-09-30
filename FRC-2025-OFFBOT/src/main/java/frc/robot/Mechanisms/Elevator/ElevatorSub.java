
package frc.robot.Mechanisms.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Mechanisms.MechanismsConstants.ElevatorConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class ElevatorSub extends NetworkSubsystem{

    private final TalonFX leader, follower;

    private final TalonFXConfiguration LeaderConfig, FollowerConfig;


    public ElevatorSub () {
        super("ElevatorSubsystem", false);

        leader = new TalonFX(ElevatorConstants.Leader_ID);
        follower = new TalonFX(ElevatorConstants.Follower_ID);

        LeaderConfig = new TalonFXConfiguration();
        FollowerConfig = new TalonFXConfiguration();

        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);

        LeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        LeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        LeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        FollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        FollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leader.getConfigurator().apply(LeaderConfig);
        follower.getConfigurator().apply(FollowerConfig);

    }

    public void runMot(double speed){
        leader.set(speed);
        follower.set(speed);
    }

    public void StopMotors (){

        leader.stopMotor();
        follower.stopMotor();
    }



    @Override
    public void NetworkPeriodic() {}
    
}
