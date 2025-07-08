package lib.ForgePlus.NetworkTableUtils;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class NTSendableChooser <T>{

    private final SendableChooser<T> chooser;
    private final String table;
    private final String key;
    private boolean hasDefault = false;
    private T defaultValue;

    public NTSendableChooser(String table, String key) {
        this.table = Objects.requireNonNull(table, "Table cannot be null");
        this.key = Objects.requireNonNull(key, "Key cannot be null");
        chooser = new SendableChooser<>();
    }

    public SendableChooser<T> get(){
        return chooser;
    }

    public NTSendableChooser<T> setDefault(String name, T value) {
        hasDefault = true;
        this.defaultValue = Objects.requireNonNull(value, "Default value cannot be null");
        chooser.setDefaultOption(name, Objects.requireNonNull(defaultValue, "Default value cannot be null"));
        return this;
    }

    public NTSendableChooser<T> add(String name, T value) {

        if (!hasDefault) {
            System.err.println("[NTSendableChooser] Warning: Adding options before setting default!");
        }

        chooser.addOption(name, Objects.requireNonNull(value, "Option value cannot be null"));
        return this;
    }

    public void publish() {
        NTPublisher.publish(table, key, chooser);
    }

    public T getSelected() {
        return chooser.getSelected();
    }

    public T getDefault() {
        return hasDefault ? defaultValue : null;
    }

    public String getSelectedName(){
        return chooser.getSelected() != null ? chooser.getSelected().toString() : "No selection";
    }
}
