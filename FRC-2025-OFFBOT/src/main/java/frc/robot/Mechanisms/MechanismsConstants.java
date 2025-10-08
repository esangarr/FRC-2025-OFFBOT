package frc.robot.Mechanisms;

import lib.ForgePlus.Math.Profiles.ProfileGains.CompleteFeedForwardGains;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import lib.ForgePlus.Math.Profiles.ProfileGains.CompleteFeedForwardGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.PIDFGains;

import lib.ForgePlus.Math.Profiles.ProfileGains.PIDGains;

public class MechanismsConstants {

    // Hay que poner los ID'S de motores y sensores correctos
    public class IndexerConstants{

        public static final int RightWheels_ID = 9;
        public static final int LeftWheels_ID = 10;

        public static final boolean RightInverted = false;
        public static final boolean LeftInverted = false;

        public static final int RightWheelsCurrentLimit = 0;
        public static final int LeftWheelsCurrentLimit = 0;

        public static final int DIO_PORT_SENSOR = 0;

    }

    public class IntakeConstants{

        public static final int IntAngle_ID = 11;
        public static final int IntWheels_ID = 14;

        public final static boolean AngulatorInverted = true;

        public final static int AngulatorCurrentLimit = 0;
        public final static int IntWheelsCurrentLimit = 0;

        public final static double pidTolerance = 3;

        public final static double GaerRatio = 17.9;

        public final static double intakeTolerance = 6;
        
        public static final PIDGains pidGainsUp = new PIDGains(0.0028 , 0, 0.);
        public static final PIDGains pidGainsDown = new PIDGains(0.0047, 0, 0.00035);
        public static final CompleteFeedForwardGains FFgains = new CompleteFeedForwardGains(
            0.0097, 
            0, 
            0, 
            0);

    
    }

   
    public class OutConstants {

        public final static int Wheels_ID = 17;
        public final static int arm_ID = 12;

        public final static boolean armInveerted = false;
        public final static boolean wheeelsInveerted = false;

        public final static int armCurrentLimit = 0;
        public final static int wheelsCurrentLimit = 0 ;
    
        public final static double pidTolerance = 1;

        public static final PIDGains pidGainsUp = new PIDGains(0.019, 0, 0);
        public static final PIDGains pidGainsDown = new PIDGains(0.0075, 0, 0);
    
    }

    public class ElevatorConstants {
        public static final int Leader_ID = 15;
        public static final int Follower_ID = 16;

        public static final int encoder_ID1 = 1;
        public static final int encoder_ID2 = 2;
        //                                      (Radio engrane en m) * 2PI
        public static final double gearCircunference = (0.03397 * 2 * Math.PI); 
        public static final double pulsesPerRevolution = 2048.0;  // EL encoder de 2048.0 pulsos por vuelta

        public static final double distancePerPulse = gearCircunference / pulsesPerRevolution; //Distancia recorrida por cada vuelta

        public static final double ELEVATOR_OFFSET_CENTIMETERS = 25;


        
    }

    public class ClimberConstants {

        public static final int Climber_ID = 18;
        public static final int ClimbWheel_ID = 13;

        public static final boolean cimberInverted = false;
    }

    
}
