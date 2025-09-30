package frc.robot.Mechanisms;

import lib.ForgePlus.Math.Profiles.ProfileGains.CompleteFeedForwardGains;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import lib.ForgePlus.Math.Profiles.ProfileGains.CompleteFeedForwardGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.PIDFGains;

import lib.ForgePlus.Math.Profiles.ProfileGains.PIDGains;

public class MechanismsConstants {

    // Hay que poner los ID'S de motores y sensores correctos
    public class IndexerConstants{

        public static final int RightWheels_ID = 0;
        public static final int LeftWheels_ID = 0;

        public static final boolean RightInverted = false;
        public static final boolean LeftInverted = false;

        public static final int RightWheelsCurrentLimit = 25;
        public static final int LeftWheelsCurrentLimit = 25;

        public static final int DIO_PORT_SENSOR = 0;

    }

    public class IntakeConstants{

        public static final int IntAngle_ID = 11;
        public static final int IntWheels_ID = 13;

        public final static boolean AngulatorInverted = false;

        public final static int AngulatorCurrentLimit = 0;
        public final static int IntWheelsCurrentLimit = 0;

        public final static double pidTolerance = 0;

        public final static double ffVelocity = 0;
        public final static double ffAceleration = 0;

        public final static double GaerRatio = 17.9;
        
        public static final PIDGains pidGainsUp = new PIDGains(0, 0, 0);
        public static final PIDGains pidGainsDown = new PIDGains(0, 0, 0);
        public static final CompleteFeedForwardGains FFgains = new CompleteFeedForwardGains(
            0, 
            0, 
            0, 
            0);

    
    }

   
    public class OutConstants {

        public final static int Wheels_ID = 0;
        public final static int arm_ID = 0;


        public final static boolean armInveerted = false;
        public final static boolean wheeelsInveerted = false;

        public final static int armCurrentLimit = 25;
        public final static int wheelsCurrentLimit = 25;
    
        public final static double pidTolerance = 0;
        public final static double armTolerance = 0;

        public final static double ffVelocity = 0;
        public final static double ffAceleration = 0;

        public static final CompleteFeedForwardGains FFgains = new CompleteFeedForwardGains(0, 0, 0, 0);
        public static final PIDGains pidGainsUp = new PIDGains(0, 0, 0);
        public static final PIDGains pidGainsDown = new PIDGains(0, 0, 0);
    
        public static final double MMAcceleration = 5000; //this is the medium point
        public static final double MMCruiseVelocity = 5000;
    }

    public class ElevatorConstants {
        public static final int Leader_ID = 15;
        public static final int Follower_ID = 16;
        
    }

    public class ClimberConstants {}

    
}
