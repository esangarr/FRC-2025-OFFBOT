package lib.ForgePlus.Equals;

/**
 * Represents a range of values
 */
public class Domain{

    /**
     * Represents the domain value
     */
    public enum DomainLimit{
        MIN, MAX
    }

    public enum DomainEdge{
        FULL, EMPTY
    }

    private double min, max;

    private DomainEdge minEdge;
    private DomainEdge maxEdge;

    /**
     * Represents a range zone
     * @param min the minimum value
     * @param max the maximum value 
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     */
    public Domain(double min , double max){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");
        }

        this.min = min;
        this.max = max;

        this.maxEdge = DomainEdge.FULL;
        this.minEdge = DomainEdge.FULL;
    }

    public Domain(double min, DomainEdge minEdge, double max, DomainEdge maxEdge){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");
        }

        this.min = min;
        this.max = max;

        this.minEdge = minEdge;
        this.maxEdge = maxEdge;
    }

    /**
     * Set the minimum value of the domain
     * @param min the minimum value
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     */
    public void setMin(double min){
        if (min > this.max) {
            throw new IllegalArgumentException("The minimum value must be less than or equal to the maximum value");
        }
        this.min = min;
    }

    /**
     * Set the maximum value of the domain
     * @param max the maximum value
     * @throws IllegalArgumentException if the maximum value is less than the minimum value
     */
    public void setMax(double max){
        if (max < this.min) {
            throw new IllegalArgumentException("The maximum value must be greater than or equal to the minimum value");
        }
        this.max = max;
    }

    /**
     * Set the domain
     * @param min the minimum value
     * @param max the maximum value
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     * @throws IllegalArgumentException if the domain is set to an empty domain
     */
    public void set(double min, double max){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");
            
        }

        if (min == 0 && max == 0){
            throw new IllegalArgumentException("The domain can't be empty!, use empty domain instead");
            
        }

        this.min = min;
        this.max = max;
    }

    
    /**
     * Gets the nearest domain to a value
     * @param value the value to check
     * @return the nearest domain
     */
    public DomainLimit nearest(double value){
        if(Math.abs(value - min) < Math.abs(value - max)){
            return DomainLimit.MIN;
        }else{
            return DomainLimit.MAX;
        }
    }

    /**
     * Check if a value is at half of the domain
     * @param value the value to check
     * @return true if the value is at half
     */
    public boolean atHalf(double value){
        return value == min + (max - min) / 2;
    }

    /**
     * Gets the distance to the minimum
     * @param value the value to check
     * @return the distance to the minimum
     */
    public double distanceToMin(double value) {
        return DomainUtils.distanceToLimit(value, min);
    }
    
    /**
     * Gets the distance to the maximum
     * @param value the value to check
     * @return the distance to the maximum
     */
    public double distanceToMax(double value) {
        return DomainUtils.distanceToLimit(value, max);
    }

    /**
     * Gets the minimum value of the domain
     * @return the minimum value
     */
    public double minValue(){
        return min;
    }
    /**
     * Gets the maximum value of the domain
     * @return the maximum value
     */
    public double maxValue(){
        return max;
    }

    /**
     * Check if a value is in the domain
     * @param value the value to check
     * @return true if the value is in the domain
     */
    public boolean inRange(double value){
        
        boolean minBool;
        boolean maxBool;

        if (minEdge == DomainEdge.EMPTY) {
            minBool = value > min;
        }else{
            minBool = value >= min;
        }

        if (maxEdge == DomainEdge.EMPTY) {
            maxBool = value < max;
        }else{
            maxBool = value <= max;
        }

        return  minBool && maxBool;
    }

    /**
     * Checks if a value is in the max of the domain
     * @param value the value to check
     * @return true if the value is at maximum
     */
    public boolean inMax(double value){
        boolean maxBool;

        if (maxEdge == DomainEdge.EMPTY) {
            maxBool = value < max;
        }else{
            maxBool = value <= max;
        }

        return maxBool;
    }

    /**
     * Checks if a value is in the min of the domain
     * @param value the value to check
     * @return true if the value is at minimum
     */
    public boolean inMin(double value){
        boolean minBool;

        if (minEdge == DomainEdge.EMPTY) {
            minBool = value > max;
        }else{
            minBool = value >= max;
        }

        return minBool;
    }

    /**
     * Check if a value is out of the domain
     * @param value the value to check
     * @return true if the value is out of the domain
     */
    public boolean outOfBounds(double value){
        return !inRange(value);
    }

}