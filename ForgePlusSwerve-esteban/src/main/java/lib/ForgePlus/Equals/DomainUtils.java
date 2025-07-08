package lib.ForgePlus.Equals;

/**
 * Represents a range of values and provides utility methods to check if a value is in the range
 */
public class DomainUtils{

    /**
     * Check if a value is in a range
     * @param value the value to check
     * @param min the minimum value
     * @param max the maximum value
     * @return true if the value is in the range
     */
    public static boolean inRange(double value, double min, double max){
        
        return value >= min && value <= max;
    }

    /**
     * Gets the distance to a limit
     * @param value the value to check
     * @param limit the limit
     * @return the distance to the limit
     */
    public static double distanceToLimit(double value, double limit) {
        return Math.abs(limit - value);
    }

    /**
     * Check if a value is out of a range
     * @param value the value to check
     * @param min the minimum value
     * @param max the maximum value
     * @return true if the value is out of the range
     */
    public static boolean outOfRange(double value, double min, double max){
        return !inRange(value, min, max);
    }

}