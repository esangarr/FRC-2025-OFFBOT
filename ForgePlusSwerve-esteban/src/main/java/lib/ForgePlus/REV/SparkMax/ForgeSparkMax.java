package lib.ForgePlus.REV.SparkMax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The ForgeSparkMax class is a wrapper for the REV SparkMax motor controller.
 * It provides methods to configure the motor controller and send data to the SmartDashboard.
 */
public class ForgeSparkMax extends SparkMax implements Sendable{


    @FunctionalInterface
    public interface EncoderValue{
        /**
         * Gets the value of the encoder.
         * @return the value of the encoder
         */
        double getRead();

        default EncoderValue withReduction(double reduction){
            return ()-> getRead() / reduction;
        }

        default EncoderValue toRadians(){
            return ()-> Units.rotationsToRadians(getRead());
        }

        default EncoderValue toDegrees(){
            return ()-> Units.rotationsToDegrees(getRead());
        }

        default EncoderValue toRadiansPerSecond(){
            return ()-> Units.rotationsPerMinuteToRadiansPerSecond(getRead());
        }

        default EncoderValue withConversion(double conversion){
            return ()-> getRead() * conversion;
        }

        default EncoderValue withOffset(double offset){
            return ()-> getRead() + offset;
        }

        default EncoderValue invert(){
            return ()-> -getRead();
        }
      
    }

    private SparkBaseConfig config;

    private RelativeEncoder encoder;

    private String name;

    /**
     * Constructor for the ForgeSparkMax class with default motor type (kBrushless)
     * @param deviceID the device ID of the motor controller
     * 
     */
    public ForgeSparkMax(int deviceID) {
        super(deviceID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        this.encoder = super.getEncoder();
        this.name = "Spark " + deviceID;
    }

    /**
     * Constructor for the ForgeSparkMax class with default motor type (kBrushless)
     * @param deviceID the device ID of the motor controller
     * @param name the name of the motor controller
     * 
     */
    public ForgeSparkMax(int deviceID, String name) {
        super(deviceID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        this.encoder = super.getEncoder();
        this.name = name;
    }

    /**
     * Constructor for the ForgeSparkMax class
     * @param deviceID the device ID of the motor controller
     * @param motorType the type of motor controller
     * @param name the name of the motor controller
     * 
     */
    public ForgeSparkMax(int deviceID, MotorType motorType) {
        super(deviceID, motorType);
        config = new SparkMaxConfig();
        this.encoder = super.getEncoder();
    }

    /**
     * Constructor for the ForgeSparkMax class
     * @param deviceID the device ID of the motor controller
     * @param motorType the type of motor controller
     * 
     */
    public ForgeSparkMax(int deviceID, MotorType motorType, String name) {
        super(deviceID, motorType);
        config = new SparkMaxConfig();
        this.encoder = super.getEncoder();
        this.name = name;
    }

    /**
     * Sets the name of the motor controller
     * @param name the name to set
     * 
     */
    public void setName(String name){
        this.name = name;
    }

    @Override
    public void initSendable(SendableBuilder builder){

        builder.setSmartDashboardType("SparkMax");

        builder.addStringProperty("MOTOR", ()-> name, this::setName);

        builder.addIntegerProperty("CAN ID", ()-> super.getDeviceId(), null);

        builder.addBooleanProperty("isConnected", ()-> isConnected(), null);
    }

    /**
     * Flash the configuration of the motor controller
     * @param motorInverted true to invert the motor direction, false to not invert
     * @param idleMode the idle mode to set
     * @param currentLimit the current limit to set
     * @param voltageCompensation true to enable voltage compensation, false to disable
     * 
     */
    public void flashConfiguration(boolean motorInverted, IdleMode idleMode, int currentLimit, boolean voltageCompensation){
        super.setCANTimeout(250);

        encoder.setPosition(0);

        config.inverted(motorInverted).
        idleMode(idleMode).
        smartCurrentLimit(currentLimit);

        if (voltageCompensation) {
            config.voltageCompensation(12);
        }

        config.encoder.
        uvwAverageDepth(2).
        uvwMeasurementPeriod(10);

        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        super.setCANTimeout(0);

    }

    /**
     * Set the voltage compensation of the motor controller
     * @param voltage the voltage to set
     * 
     */
    public void setVoltageCompensation(double voltage){
        super.setCANTimeout(250);
        config.voltageCompensation(voltage);
        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        super.setCANTimeout(0);
    }

    /**
     * Set the smart current limit of the motor controller
     * @param currentLimit the current limit to set
     * 
     */
    public void setSmartCurrentLimit(int currentLimit){
        super.setCANTimeout(250);
        config.smartCurrentLimit(currentLimit);
        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        super.setCANTimeout(0);
    }

    /**
     * Set the idle mode of the motor controller
     * @param idleMode the idle mode to set
     * 
     */
    public void setIdleMode(IdleMode idleMode){
        super.setCANTimeout(250);
        config.idleMode(idleMode);
        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        super.setCANTimeout(0);
    }

    /**
     * Invert the motor direction
     * @param inverted true to invert the motor direction, false to not invert
     * 
     */
    @Override
    public void setInverted(boolean inverted){
        super.setCANTimeout(250);
        config.inverted(inverted);
        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        super.setCANTimeout(0);
    }

    /**
     * Commands a set motor speed 
     * @param speed the speed to set
     * 
     */
    public Command speedCommand(double speed, Subsystem requirements){
        return Commands.runEnd(()-> super.set(speed), ()-> super.set(0), requirements);
    }

    /**
     * Commands a set motor voltage
     * @param volts the voltage to set
     * 
     */
    public Command voltageCommand(double volts, Subsystem requirements){
        return Commands.runEnd(()-> super.setVoltage(volts), ()-> super.set(0), requirements);
    }

    /**
     * Checks if the motor controller is connected
     * @return true if the motor controller is connected, false otherwise
     */
    public boolean isConnected(){
        return !(super.getLastError() == REVLibError.kCANDisconnected);
    }

    /**
     * Checks if the motor is moving
     * @return true if the motor is moving, false otherwise
     * 
     */
    public boolean isMoving(){
        return super.get() != 0 || super.getBusVoltage() != 0;
    }

    /**
     * Gets the current position of the motor
     * @return the current position of the motor
     */
    public EncoderValue getPosition(){
        return ()-> encoder.getPosition();
    }

    /**
     * Sets the position of the motor
     * @param position the position to set
     * 
     */
    public void setPosition(double position){
        encoder.setPosition(position);
    }

    /**
     * Gets the current velocity of the motor
     * @return the current velocity of the motor
     * 
     */
    public EncoderValue getVelocity(){
        return ()-> encoder.getVelocity();
    }

    /**
     * Gets the current encoder of the motor
     * @return the current encoder of the motor
     */
    public RelativeEncoder getEncoder(){
        return encoder;
    }

    /**
     * Resets the position of the motor to 0
     */
    public void resetPosition(){
        encoder.setPosition(0);
    }

}
