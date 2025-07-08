package frc.robot.DriveTrain;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(28); 
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(31.7); 
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double ROBOTMASSKG = 57.45;
    public static final double ROBOTMOI = 5.16;

    public static final PathConstraints fastPathConstraints = new PathConstraints(
        4.5,
        4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    public static final PathConstraints normalPathConstraints = new PathConstraints(
        3.0,
        3.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );
    
    public static final class frontLeft{

        public static final int DrivePort = 1; 
        public static final int TurnPort = 2; 
        public static final int EncPort = 19;
        public static final double offset = 0.08;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class frontRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 20; 
        public static final double offset = -0.47; 
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 5; 
        public static final int TurnPort = 6; 
        public static final int EncPort = 21; 
        public static final double offset = 0.49;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 7; 
        public static final int TurnPort = 8; 
        public static final int EncPort = 22; 
        public static final double offset = -0.43; 
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }
}
