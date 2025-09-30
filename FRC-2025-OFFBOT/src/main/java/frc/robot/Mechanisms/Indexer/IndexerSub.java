/*package frc.robot.Mechanisms.Indexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Mechanisms.MechanismsConstants.IndexerConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class IndexerSub extends NetworkSubsystem{

    private final ForgeSparkMax leftMot, rightMot;

    private final DigitalInput BeamSensor;

    //:v
    public IndexerSub (){
        super("IndexSubsystem", false);

        BeamSensor = new DigitalInput(IndexerConstants.DIO_PORT_SENSOR); 

        leftMot = new ForgeSparkMax(IndexerConstants.RightWheels_ID, "IndexerLeftWheels");
        rightMot = new ForgeSparkMax(IndexerConstants.RightWheels_ID, "IndexerRightWheels");

        leftMot.flashConfiguration(
            IndexerConstants.LeftInverted,
            IdleMode.kCoast,
            IndexerConstants.LeftWheelsCurrentLimit,
            false);
        
        rightMot.flashConfiguration(
            IndexerConstants.RightInverted,
            IdleMode.kCoast,
            IndexerConstants.RightWheelsCurrentLimit,
            false);

    }

    @Override
    public void NetworkPeriodic(){ }

    public void runWheels(double speed){
        leftMot.set(speed);
        rightMot.set(speed);
    }

    @AutoNetworkPublisher(key = "hasPiece")
    public boolean hasPiece(){
        return BeamSensor.get();
    }

    public boolean isWheelSpinning(){
        boolean spinning = (rightMot.get() != 0 && leftMot.get() != 0);

        return spinning;
    }

    public void stop(){
        rightMot.stopMotor();
        leftMot.stopMotor();
    }





    
}*/
