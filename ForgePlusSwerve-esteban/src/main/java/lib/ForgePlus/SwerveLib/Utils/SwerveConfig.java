package lib.ForgePlus.SwerveLib.Utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SwerveConfig{
    
    /**
     * Class that stores the most common motors used for a swerve drive
     */
    public static class SingleMotorGearBoxes{
        
        public static DCMotor kNEO = DCMotor.getNEO(1);
        public static DCMotor kKRAKEN = DCMotor.getKrakenX60(1);

    }

    /**
     * Class for storing swerve module reductions
     * @param driveReduction drive reduction
     * @param turnReduction turn reduction
     */
    public record ModuleReductions(double driveReduction, double turnReduction) {}

    public record Wheel(double radiusInches) {
        public double radiusMeters(){return Units.inchesToMeters(radiusInches);}

        public double diameterMeters(){return radiusMeters() * 2;}

        public double diameterInches(){return radiusInches * 2;}
    }

    /**
     * Class for storing swerve module current limits in Ampers
     * @param driveCurrentAmps
     * @param turnCurrentAmps
     */
    public record ModuleCurrentLimits(int driveCurrentAmps, int turnCurrentAmps) {}
    
}
