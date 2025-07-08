package lib.ForgePlus.Equals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The {@code TwoDimensionalSpace} class defines a 2D space using X and Y coordinates,
 * along with a tolerance that represents the allowable deviation in both axes.
 * This class provides methods to check if a given point (or pose) is within this defined space.
 */
public class TwoDimensionalSpace{

    private Domain XaxisDomain;
    private Domain YaxisDomain;

    private double tolerance;

    private Translation2d translation;

    /**
     * Constructs a 2D space based on the X and Y coordinates with a given tolerance.
     * The tolerance defines a percentage range around the coordinates.
     * 
     * @param x The X-coordinate of the space.
     * @param y The Y-coordinate of the space.
     * @param tolerance The allowable deviation in both X and Y axes (percentage of the coordinates).
     */
    public TwoDimensionalSpace(double x, double y, double tolerance){

        this.tolerance = tolerance;
        this.translation = new Translation2d(x, y);

        double percentageInX = Math.abs(x * tolerance);
        double percentageInY = Math.abs(y * tolerance);

        XaxisDomain = new Domain(x - percentageInX, x + percentageInX);
        YaxisDomain = new Domain(y - percentageInY, y + percentageInY);
    }

    /**
     * Constructs a 2D space based on a {@code Translation2d} object and a given tolerance.
     * 
     * @param xy The 2D coordinate (X and Y) as a {@code Translation2d} object.
     * @param tolerance The allowable deviation in both X and Y axes (percentage of the coordinates).
     */
    public TwoDimensionalSpace(Translation2d xy, double tolerance){

        this.tolerance = tolerance;
        this.translation = xy;

        double percentageInX = xy.getX() * tolerance;
        double percentageInY = xy.getY() * tolerance;

        XaxisDomain = new Domain(xy.getX() - percentageInX, xy.getX() + percentageInX);
        YaxisDomain = new Domain(xy.getY() - percentageInY, xy.getY() + percentageInY);
    }

    /**
     * Returns the {@code Translation2d} representing the center of the 2D space.
     * 
     * @return The center of the 2D space as a {@code Translation2d}.
     */
    public Translation2d asTranslation(){
        return translation;
    }

    /**
     * Returns the tolerance value used for this 2D space.
     * 
     * @return The tolerance value.
     */
    public double getTolerance(){
        return tolerance;
    }

    /**
     * Returns the range of X coordinates (min and max) for this 2D space as a {@code Translation2d}.
     * 
     * @return The range of X coordinates.
     */
    public Translation2d getXRange(){
        return new Translation2d(XaxisDomain.minValue(), XaxisDomain.maxValue());
    }
    
    /**
     * Returns the range of Y coordinates (min and max) for this 2D space as a {@code Translation2d}.
     * 
     * @return The range of Y coordinates.
     */
    public Translation2d getYRange(){
        return new Translation2d(YaxisDomain.minValue(), YaxisDomain.maxValue());
    }

    /**
     * Checks if a given {@code Pose2d} is within the defined 2D space.
     * 
     * @param space The {@code Pose2d} to check.
     * @return {@code true} if the pose is within the space, {@code false} otherwise.
     */
    public boolean atSpace(Pose2d space){
        return XaxisDomain.inRange(space.getX()) && YaxisDomain.inRange(space.getY());
    }


}