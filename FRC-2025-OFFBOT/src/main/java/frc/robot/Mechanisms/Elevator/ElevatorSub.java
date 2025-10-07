
package frc.robot.Mechanisms.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MecaCommands.ElevatorCommands.ElevatorCommands;
import frc.robot.Mechanisms.MechanismsConstants.ElevatorConstants;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;

public class ElevatorSub extends NetworkSubsystem{

    private TalonFX leader, follower;

    private TalonFXConfiguration LeaderConfig, FollowerConfig;

    private Encoder encoder;




    public ElevatorSub () {
        super("ElevatorSubsystem", false);

        leader = new TalonFX(ElevatorConstants.Leader_ID);
        follower = new TalonFX(ElevatorConstants.Follower_ID);


        LeaderConfig = new TalonFXConfiguration();
        FollowerConfig = new TalonFXConfiguration();

        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake );

        LeaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        LeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        LeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        FollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        FollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leader.getConfigurator().apply(LeaderConfig);
        follower.getConfigurator().apply(FollowerConfig);

        encoder = new Encoder(1, 2, true, Encoder.EncodingType.k4X);
        //encoder.setDistancePerPulse(ElevatorConstants.distancePerPulse);


    }

    public double Ticks(){
        return encoder.getDistancePerPulse();
    }

    public double getDistanceCm(){
        return ((124.5/17440) * encoder.getDistance()) + 63.5 ;
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
    public void NetworkPeriodic() {
       NTPublisher.publish("Distance", getTableKey(), getDistanceCm());
       NTPublisher.publish("Ticks", getTableKey(), encoder.getDistancePerPulse());
    }
    }
    

