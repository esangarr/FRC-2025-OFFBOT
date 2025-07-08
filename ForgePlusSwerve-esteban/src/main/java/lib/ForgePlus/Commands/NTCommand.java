package lib.ForgePlus.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;

public interface NTCommand{

    /**
     * Publishes a command to NetworkTables with the specified name.
     *
     * @param name The name under which the command will be published.
     * @throws IllegalArgumentException if the command is null.
     */
    public default void publish(String name) {
        if (this instanceof Command command) {
            NTPublisher.publish("NTCommands", name, command);
        } else {
            throw new IllegalArgumentException("Class is not a Command instance");
        }
    }

}
