package lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to mark fields or methods as NetworkCommands.
 * The {@code value} parameter is an optional identifier for the NetworkCommand, typically used as the key
 * for a NetworkTables entry or some identifier within the system.
 * 
 * <p>This annotation can be applied to fields or methods that represent commands or actions that interact with
 * the NetworkTables system. The {@code value} parameter provides a customizable identifier for each command.</p>
 */
@Target({ElementType.FIELD, ElementType.METHOD})
@Retention(RetentionPolicy.RUNTIME)
public @interface NetworkCommand{

    /**
     * The identifier or key for the NetworkCommand.
     * This can be used to uniquely identify the command within NetworkTables or the system.
     * 
     * @return The identifier for the NetworkCommand.
     */
    String value() default "";
}
