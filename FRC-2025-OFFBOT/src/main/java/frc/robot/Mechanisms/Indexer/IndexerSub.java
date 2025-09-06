package frc.robot.Mechanisms.Indexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Mechanisms.MechanismsConstants.IndexerConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class IndexerSub extends NetworkSubsystem{

    private final ForgeSparkMax rightWheels, leftWheels;

    private final DigitalInput BeamSensor;

    //:v
    public IndexerSub (){
        super("IndexSubsystem", false);

        BeamSensor = new DigitalInput(IndexerConstants.DIO_PORT_SENSOR); 

        rightWheels = new ForgeSparkMax(IndexerConstants.RightWheels_ID, "IndexerRightWheels");
        leftWheels = new ForgeSparkMax(IndexerConstants.LeftWheels_ID, "IndexerLeftWheels");

        rightWheels.flashConfiguration(
            IndexerConstants.RightInverted,
            IdleMode.kCoast,
            IndexerConstants.RightWheelsCurrentLimit,
            false);

        leftWheels.flashConfiguration(
            IndexerConstants.LeftInverted,
            IdleMode.kCoast,
            IndexerConstants.LeftWheelsCurrentLimit,
            false);
     
    }

    @Override
    public void NetworkPeriodic(){ }

    public void runWheels(double speed){
        rightWheels.set(speed);
        leftWheels.set(speed);
    }

    @AutoNetworkPublisher(key = "hasPiece")
    public boolean hasPiece(){
        return BeamSensor.get();
    }

    public boolean isWheelSpinning(){
        boolean spinning = (rightWheels.get() != 0 && leftWheels.get() != 0);

        return spinning;
    }

    public void stop(){
        rightWheels.stopMotor();
        leftWheels.stopMotor();
    }





    
}
