package lib.ForgePlus.NetworkTableUtils;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A utility class for creating and managing a tunable number in NetworkTables.
 * This class allows for subscribing to a value, updating it, and publishing changes back to NetworkTables.
 * The value can be modified from both the robot code and external sources (e.g., a dashboard).
 * 
 * <p>Note: To ensure the class functions correctly, the {@link #update()} method must be called periodically
 * (e.g., in a periodic loop or command). This ensures that the latest value from NetworkTables is retrieved
 * and the local value is kept up to date.</p>
 */
public class NTTunnableNumber {

    private final DoubleSubscriber subscriber;
    private final DoublePublisher publisher;
    private final NetworkTableEntry entry;
    private double lastValue;

    /**
     * Creates a new {@code NetworkTunnableNumber} that interacts with a NetworkTables entry to store and 
     * retrieve a tunable double value.
     * The value will be subscribed and published to the given key, with a default value if the key does not exist.
     *
     * @param key The NetworkTables key where the tunable number will be stored.
     * @param defaultValue The default value to use if the key does not exist in NetworkTables.
     */
    public NTTunnableNumber(String key, double defaultValue){
        var instance = NetworkTableInstance.getDefault();
        this.entry = instance.getEntry(key);
        
        var topic = instance.getDoubleTopic(key);
        this.subscriber = topic.subscribe(entry.exists() ? entry.getDouble(defaultValue) : defaultValue);
        this.publisher = topic.publish();
        this.publisher.set(entry.exists() ? entry.getDouble(defaultValue) : defaultValue);
        this.lastValue = entry.exists() ? entry.getDouble(defaultValue) : defaultValue;
    }

    /**
     * Updates the stored value by reading the latest value from the NetworkTables subscriber.
     * 
     * <p>It is important to call this method periodically (e.g., in a periodic loop or command) to ensure that
     * the latest value is fetched from NetworkTables and the local value is kept up to date.</p>
     */
    public void update() {
        lastValue = subscriber.get();
    }

    /**
     * Retrieves the current value of the tunable number.
     *
     * @return The current value of the tunable number.
     */
    public double get() {
        return lastValue;
    }

    /**
     * Sets a new value for the tunable number and publishes it to NetworkTables.
     * The value will only be published if it is different from the last published value.
     *
     * @param value The new value to set and publish.
     */
    public void set(double value) {
        if (value != lastValue) {
            lastValue = value;
            publisher.set(value);
            entry.setDouble(value);
        }
    }

    /**
     * Checks if the tunable number has changed since the last update.
     *
     * @return {@code true} if the value has changed, {@code false} otherwise.
     */
    public boolean hasChanged() {
        return subscriber.get() != lastValue;
    }
}
