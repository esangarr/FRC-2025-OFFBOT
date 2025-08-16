package frc.robot.Mechanisms.Indexer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Mechanisms.MechanismsConstants.IndexerConstants;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.NetworkSubsystem;

public class IndexerSub extends NetworkSubsystem{

    private final SparkMax rightWheels, leftWheels;
    private final SparkMaxConfig rightWheelsConfig, leftWheelsConfig;

    private final DigitalInput BeamSensor;


    public IndexerSub (){
        super("IndexSubsystem", false);

        BeamSensor = new DigitalInput(IndexerConstants.DIO_PORT_SENSOR); 

        rightWheels = new SparkMax(IndexerConstants.RightWheels_ID, MotorType.kBrushless);
        leftWheels = new SparkMax(IndexerConstants.LeftWheels_ID, MotorType.kBrushless);

        rightWheelsConfig = new SparkMaxConfig();
        leftWheelsConfig = new SparkMaxConfig();

        configureMotors();
        
    }

    public void configureMotors () {
        leftWheels.setCANTimeout(250);
        rightWheels.setCANTimeout(250);

        leftWheelsConfig.inverted(IndexerConstants.LeftInverted).idleMode(IdleMode.kCoast).smartCurrentLimit(IndexerConstants.RightWheelsCurrentLimit);
        rightWheelsConfig.inverted(IndexerConstants.RightInverted).idleMode(IdleMode.kCoast).smartCurrentLimit(IndexerConstants.LeftWheelsCurrentLimit);

        rightWheels.configure(rightWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftWheels.configure(leftWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightWheels.setCANTimeout(0);
        leftWheels.setCANTimeout(0);

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
