package lib.ForgePlus.Math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Represents 3D standard deviations for use in Kalman filters or pose estimation.
 */
public class StandardDeviations implements Sendable{

    private double x;
    private double y;
    private double z;

    /**
     * Constructs a StandardDeviations object with specified values for x, y, and z.
     *
     * @param x The standard deviation X
     * @param y The standard deviation Y
     * @param z The standard deviation Z
     */
    public StandardDeviations(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("StandardDeviations");
        
        builder.addDoubleProperty("X", this::getX, this::setX);
        builder.addDoubleProperty("Y", this::getY, this::setY);
        builder.addDoubleProperty("Z", this::getZ, this::setZ);
    }

    /**
     * Constructs a StandardDeviations object with all standard deviations set to zero.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the standard deviation for x.
     *
     * @return The standard deviation X
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the standard deviation for z.
     *
     * @return The standard deviation Z
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets the standard deviations for x, y, and z.
     *
     * @param x The standard deviation X
     * @param y The standard deviation Y
     * @param z The standard deviation Z
     */
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Sets the standard deviation for x.
     *
     * @param x The standard deviation X
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Sets the standard deviation for y.
     *
     * @param y The standard deviation Y
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Sets the standard deviation for z.
     *
     * @param z The standard deviation Z
     */
    public void setZ(double z) {
        this.z = z;
    }

    /**
     * Returns a 3D vector representation of the standard deviations.
     *
     * @return A Matrix representing the standard deviations as a vector.
     */
    public Matrix<N3, N1> asVector(){
        return VecBuilder.fill(x, y, z);
    }
}
