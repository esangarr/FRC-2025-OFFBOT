package lib.ForgePlus.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;

/**
 * CommandChooser is a utility class that allows for the creation of a command chooser
 * that can be used to select and run commands from a list of options.
 * It can publish the chooser to SmartDashboard or NetworkTables.
 */
public class CommandChooser{

    /**
     * A record to hold the name and command for a command option in the chooser.
     * 
     * @param name The name of the command option.
     * @param command The command associated with this option.
     */
    public static final record CommandOption(String name, Command command){}

    private final SendableChooser<Command> chooser;

    /**
     * Creates a CommandChooser with the specified table and key.
     * 
     * @param table The NetworkTable to publish the chooser to.
     * @param key The key under which the chooser will be published.
     * @param defaultCommand The default command to be selected in the chooser.
     */
    public CommandChooser(String table, String key, CommandOption defaultCommand){
        this.chooser = new SendableChooser<>();

        if (defaultCommand != null) {
            chooser.setDefaultOption(defaultCommand.name(), defaultCommand.command());
        }

        NTPublisher.publish(table, key, chooser);
    }

    /**
     * Creates a CommandChooser with the specified key and default command.
     * 
     * @param key The key under which the chooser will be published to SmartDashboard.
     * @param defaultCommand The default command to be selected in the chooser.
     */
    public CommandChooser(String key, CommandOption defaultCommand) {

        this.chooser = new SendableChooser<>();

        if (defaultCommand != null) {
            chooser.setDefaultOption(defaultCommand.name(), defaultCommand.command());
        }

        SmartDashboard.putData(key, chooser);
    }

    /**
     * Creates a CommandChooser with the specified table, key, and commands.
     * 
     * @param table The NetworkTable to publish the chooser to.
     * @param key The key under which the chooser will be published.
     * @param defaultCommand The default command to be selected in the chooser.
     * @param commands The commands to be added to the chooser.
     * 
     * @throws IllegalArgumentException if any command is null.
     */
    public CommandChooser(String table, String key,CommandOption defaultCommand, CommandOption... commands) {
        this.chooser = new SendableChooser<>();

        if (defaultCommand != null) {
            chooser.setDefaultOption(defaultCommand.name(), defaultCommand.command());
        }

        for (CommandOption command : commands) {
            if (command != null) {
                chooser.addOption(command.name(), command.command());
            } else {
                throw new IllegalArgumentException("Command cannot be null");
            }
        } 
        
        NTPublisher.publish(table, key, chooser);
    }

    /**
     * Creates a CommandChooser with the specified key and commands.
     * 
     * @param key The key under which the chooser will be published to SmartDashboard.
     * @param defaultCommand The default command to be selected in the chooser.
     * @param commands The commands to be added to the chooser.
     * 
     * @throws IllegalArgumentException if any command is null.
     * 
     */
    public CommandChooser(String key,CommandOption defaultCommand, CommandOption... commands) {
        this.chooser = new SendableChooser<>();

        if (defaultCommand != null) {
            chooser.setDefaultOption(defaultCommand.name(), defaultCommand.command());
        }

        for (CommandOption command : commands) {
            if (command != null) {
                chooser.addOption(command.name(), command.command());
            } else {
                throw new IllegalArgumentException("Command cannot be null");
            }
        } 
        
        SmartDashboard.putData(key, chooser);
    }

    /**
     * Adds a command to the chooser.
     * @param command The command to be added.
     * @throws IllegalArgumentException if the command is null.
     */
    public CommandChooser addCommand(CommandOption command) {
        if (command != null) {
            chooser.addOption(command.name(), command.command());
        } else {
            throw new IllegalArgumentException("Command cannot be null");
        }

        return this;
    }

    /**
     * Runs the selected command from the chooser.
     * If no command is selected, it returns a no-op command.
     * @return The selected command, or a no-op command if none is selected.
     */
    public Command run(){
        if (chooser.getSelected() == null) {
            return Commands.none();
        }
        return chooser.getSelected();
    }

    public SendableChooser<Command> getChooser() {
        return chooser;
    }
}
