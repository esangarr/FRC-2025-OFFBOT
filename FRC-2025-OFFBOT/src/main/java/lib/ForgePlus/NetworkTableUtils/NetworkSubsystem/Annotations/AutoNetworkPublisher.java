package lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to mark methods that automatically publish data to NetworkTables.
 * The {@code key} parameter specifies the NetworkTables key where the method's output will be published.
 * 
 * <p>This annotation can be applied to methods that are intended to automatically publish their return values to
 * a specific NetworkTables entry. The {@code key} provides the name of the entry in NetworkTables where the data
 * will be published.</p>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AutoNetworkPublisher{
    /**
     * The NetworkTables key where the method's return value will be published.
     *
     * @return The key for the NetworkTables entry.
     */
    String key();
}
