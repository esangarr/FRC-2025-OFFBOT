package frc.robot.Mechanisms.Outake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class OutConstants {
    final static int Wheel_ID = 1;
    
    final static int CANWrist_ID = 1;
    final static int KrakenWrist_ID = 2;
    
    final static double WristTolerance = 0.3472;
    final static double WristAmp = 25; //Amp Limit for wrist. Idk the actual vale
    final static double WheelsAmp = 25; //Amp Limit for wrist

    // PID pal wrist 
    public static final double CANWrist_kP = 0.1;
    public static final double CANWrist_kI = 0.0;
    public static final double CANWrist_kD = 0.0;
    public static final double Wrist_IZone = 0.0;
    public static final double Wrist_MaxOutput = 0.7;
    public static final double Wrist_MinOutput = 0.2;

    
    
    public static final double MMAcceleration = 5000; //this is the medium point
    public static final double MMCruiseVelocity = 5000;

    /*/
    public OutConstants(){
    /*set motor id's
    Gains (for the PID's)
    and positions?
    */
    VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    TalonFXConfiguration KrakenConfigs = new TalonFXConfiguration();
    TalonFXConfiguration angMotor1Config = new TalonFXConfiguration();

}