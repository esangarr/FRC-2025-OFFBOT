package lib.ForgePlus.SwerveLib.Odometer;

import edu.wpi.first.math.util.Units;

/**
* Constructs a {@code FieldObject} with the specified length and width.
* 
* @param length The length of the object.
* @param width The width of the object.
*/
public record FieldObject(double length, double width) {

    public static final FieldObject REEFSCAPE = new FieldObject(
        
    Units.inchesToMeters(690.876), Units.inchesToMeters(317));
}
