package frc.robot.Mechanisms.Indexer;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        leftMot = new ForgeSparkMax(IndexerConstants.LeftWheels_ID, "IndexerLeftWheels");
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
    public void NetworkPeriodic(){
        SmartDashboard.putNumber("Vright", getVoltageRight());
        SmartDashboard.putNumber("Vleft", getVoltageLeft());
     }

    public void runWheels(double speedRight, double speedLeft){
        leftMot.set(speedLeft);
        rightMot.set(speedRight);
    }

    @AutoNetworkPublisher(key = "VoltageLeft")
    public double getVoltageLeft(){
        return leftMot.getBusVoltage();
    }

    @AutoNetworkPublisher(key = "VoltageRight")
    public double getVoltageRight(){
        
        return rightMot.getBusVoltage();
        
    }

    @AutoNetworkPublisher(key = "CurrentLeft")
    public double getCurrentLeft(){
        return leftMot.getOutputCurrent();
    }

    @AutoNetworkPublisher(key = "CurrentRight")
    public double getCurrentRight(){
        
        return rightMot.getOutputCurrent();
        
    }

    @AutoNetworkPublisher(key = "hasPiece")
    public boolean hasPiece(){
        return BeamSensor.get();
    }

    public boolean isWheelSpinning(){
        return rightMot.get() != 0 && leftMot.get() != 0;
    }

    @AutoNetworkPublisher(key = "One Stuck")
    public Boolean isStuck(){
        return getVoltageRight() <= 7.7 || getVoltageLeft()<= 7.7;
    }

    @AutoNetworkPublisher(key = "BOTH Stuck")
    public Boolean BothStuck(){
        return getVoltageRight() <= 7.7 || getVoltageLeft()<= 7.7;
    }


    @AutoNetworkPublisher(key = "Clear")
    public Boolean isClear(){
        return getVoltageRight() > 7.7 || getVoltageLeft() > 7.7;
    }

    public void stop(){
        rightMot.stopMotor();
        leftMot.stopMotor();
    }





    
} 
