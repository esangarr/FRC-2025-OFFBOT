package frc.robot.Mechanisms.Indexer;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.MechanismsConstants.IndexerConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.REV.SparkMax.ForgeSparkMax;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class IndexerSub extends NetworkSubsystem{

    private final ForgeSparkMax leftMot, rightMot;

    private final DigitalInput BeamSensor;

    private final Debouncer timer;

    //:v
    public IndexerSub (){
        super("IndexSubsystem", false);

        BeamSensor = new DigitalInput(IndexerConstants.DIO_PORT_SENSOR); 

        leftMot = new ForgeSparkMax(IndexerConstants.LeftWheels_ID, "IndexerLeftWheels");
        rightMot = new ForgeSparkMax(IndexerConstants.RightWheels_ID, "IndexerRightWheels");

        timer = new Debouncer(0.45);

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
    public void NetworkPeriodic(){
        NTPublisher.publish("CurrentRight", getTableKey(), getCurrentRight());
        NTPublisher.publish("CurrentLeft", getTableKey(), getCurrentLeft());
        NTPublisher.publish("One Stuck", getTableKey(), isStuck());
        NTPublisher.publish("Both stuck", getTableKey(), BothStuck());
        NTPublisher.publish("Is Clear", getTableKey(), isClear());
        NTPublisher.publish("Has Piece", getTableKey(), hasPiece());
     }

    public void runWheels(double speedRight, double speedLeft){
        leftMot.set(speedLeft);
        rightMot.set(speedRight);
    }
 
    public double getCurrentLeft(){
        return leftMot.getOutputCurrent();
    }

  
    public double getCurrentRight(){
        
        return rightMot.getOutputCurrent();
        
    }

    public boolean hasPiece(){
        return BeamSensor.get();
    }

    public double rightSpin(){
        return rightMot.get();
    }

    public double leftSpin(){
        return leftMot.get();
    }

    public boolean isWheelSpinning(){
        return rightMot.get() != 0 && leftMot.get() != 0;
    }

    public Boolean isStuck(){
        return timer.calculate(getCurrentRight() >= 27 || getCurrentLeft() >= 27);
    }

    public Boolean BothStuck(){
        return timer.calculate(getCurrentRight() >= 27 && getCurrentLeft() >= 27);
    }

    public Boolean isClear(){
        return getCurrentRight() < 27 && getCurrentLeft() < 27;
    }

    public void stop(){
        rightMot.stopMotor();
        leftMot.stopMotor();
    }





    
} 
