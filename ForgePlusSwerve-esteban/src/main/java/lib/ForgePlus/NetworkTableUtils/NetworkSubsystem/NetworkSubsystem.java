package lib.ForgePlus.NetworkTableUtils.NetworkSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.AutoNetworkPublisher;
import lib.ForgePlus.NetworkTableUtils.NetworkSubsystem.Annotations.NetworkCommand;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.util.Color;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.*;
import java.util.function.Supplier;

public abstract class NetworkSubsystem extends SubsystemBase{

    private static final Set<Runnable> registeredPublishers = new HashSet<>();
    private final String tableName;

    public static final double kDefaultPeriod = 0.02;

    public NetworkSubsystem(String tableName, boolean subsystemInfo) {
        this.tableName = tableName;
        registerAnnotatedPublishers();
        registerNetworkCommands(tableName);
        NTPublisher.publish(tableName, "TableKey", getTableKey());

        if (subsystemInfo) {
            NTPublisher.publish(tableName, "Subsystem", this);
        }
        
    }

    /**
     * Gets the subsystem Key in NetworkTables
     * @return the key
     */
    public String getTableKey(){
        return tableName;
    }

    private void registerAnnotatedPublishers() {

        clearPublishers();
        
        if (!registeredPublishers.isEmpty()) {
            System.err.println("[NetworkSubsystem] WARNING: registerAnnotatedPublishers() Overloop!");
            return;
        }
    
        for (Method method : this.getClass().getDeclaredMethods()) {
            if (!method.isAnnotationPresent(AutoNetworkPublisher.class)) continue;
    
            String key = method.getAnnotation(AutoNetworkPublisher.class).key();
            
            method.setAccessible(true);
    
            Supplier<?> supplier = () -> {
                try { return method.invoke(this); }
                catch (Exception e) { e.printStackTrace(); return null; }
            };
    
            Class<?> returnType = method.getReturnType();
    
            try {
                if (returnType == Pose2d.class) {
            
                   registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Pose2d) supplier.get()));

                } else if (returnType == Pose2d[].class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Pose2d[]) supplier.get()));

                } else if (returnType == Pose3d.class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Pose3d) supplier.get()));

                } else if (returnType == Pose3d[].class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Pose3d[]) supplier.get()));

                } else if (returnType == Rotation2d.class) {
                    
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Rotation2d) supplier.get()));

                } else if (returnType == Rotation2d[].class) {
    
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Rotation2d[]) supplier.get()));
                
                } else if (returnType == Rotation3d.class) {
           
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Rotation3d) supplier.get()));

                } else if (returnType == Translation2d.class) {

                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Translation2d) supplier.get()));

                }else if(returnType == Translation2d[].class){
                    
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Translation2d[]) supplier.get()));
                }
                else if (returnType == Translation3d.class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Translation3d) supplier.get()));

                }
                else if(returnType == Translation3d[].class){
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (Translation3d[]) supplier.get()));

                }
                else if (returnType == ChassisSpeeds.class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (ChassisSpeeds) supplier.get()));

                } else if (returnType == SwerveModuleState[].class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (SwerveModuleState[]) supplier.get()));

                } else if (returnType == SwerveModulePosition[].class) {

                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (SwerveModulePosition[]) supplier.get()));

                } else if (returnType == double[].class || returnType == Double[].class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (double[]) supplier.get()));

                } else if (returnType == double.class || returnType == Double.class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (double) supplier.get()));
                    
                } else if (returnType == boolean.class || returnType == Boolean.class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (boolean) supplier.get()));

                } else if (returnType == Boolean[].class || returnType == boolean[].class) {
                    registerPublisher(()-> NTPublisher.publish(getTableKey(), key, (double[]) supplier.get()));
                } else {
                    System.err.println("[AutoNetworkPublisher] Not Supported Data type! " + returnType.getSimpleName());
                }
    
            } catch (Exception e) {
                System.err.println("[AutoNetworkPublisher] Error creating publisher at key: " + key);
                e.printStackTrace();
            }
        }
    }

    public void registerNetworkCommands(String table) {
      
        for (Field field : this.getClass().getDeclaredFields()) {
            if (field.isAnnotationPresent(NetworkCommand.class) && Command.class.isAssignableFrom(field.getType())) {
                field.setAccessible(true);
                try {
                    Command command = (Command) field.get(this);
                    if (command != null) {
                        NetworkCommand annotation = field.getAnnotation(NetworkCommand.class);
                        String path = annotation.value().isEmpty() ? "NetworkCommands/" + field.getName() : annotation.value();
                        NTPublisher.publish(table, path, command);
                    }
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }

        for (Method method : this.getClass().getDeclaredMethods()) {
            if (method.isAnnotationPresent(NetworkCommand.class) && Command.class.isAssignableFrom(method.getReturnType()) && method.getParameterCount() == 0) {
                method.setAccessible(true);
                try {
                    Command command = (Command) method.invoke(this);
                    if (command != null) {
                        NetworkCommand annotation = method.getAnnotation(NetworkCommand.class);
                        String path = annotation.value().isEmpty() ? "NetworkCommands/" + method.getName() : annotation.value();
                        NTPublisher.publish(table, path, command);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }
    
    private void registerPublisher(Runnable publisher) {
        registeredPublishers.add(publisher);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, double value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, double[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, boolean value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, boolean[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, String value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, String[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Color value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Pose2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Pose2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Pose3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Pose3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Rotation2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Rotation2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Rotation3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Rotation3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Translation2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Translation2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Translation3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Translation3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Transform2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
    * Publishes an object to NetworkTables.
    * @param key the subkey for showing on NT
    * @param value the object to publish
    */
    public final void publishOutput(String key, Transform2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Transform3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Transform3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, SwerveModulePosition value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, SwerveModulePosition[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, SwerveModuleState value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, SwerveModuleState[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, ChassisSpeeds value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, ChassisSpeeds[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publishOutput(String key, Sendable value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public double getOutput(String key, double defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public double[] getOutput(String key, double[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public boolean getOutput(String key, boolean defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public boolean[] getOutput(String key, boolean[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public String getOutput(String key, String defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public String[] getOutput(String key, String[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Color getOutput(String key, Color defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose2d getOutput(String key, Pose2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose2d[] getOutput(String key, Pose2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose3d getOutput(String key, Pose3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose3d[] getOutput(String key, Pose3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation2d getOutput(String key, Rotation2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation2d[] getOutput(String key, Rotation2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation3d getOutput(String key, Rotation3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation3d[] getOutput(String key, Rotation3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation2d getOutput(String key, Translation2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation2d[] getOutput(String key, Translation2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation3d getOutput(String key, Translation3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation3d[] getOutput(String key, Translation3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform2d getOutput(String key, Transform2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform2d[] getOutput(String key, Transform2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform3d getOutput(String key, Transform3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform3d[] getOutput(String key, Transform3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModulePosition getOutput(String key, SwerveModulePosition defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModulePosition[] getOutput(String key, SwerveModulePosition[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModuleState getOutput(String key, SwerveModuleState defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModuleState[] getOutput(String key, SwerveModuleState[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public ChassisSpeeds getOutput(String key, ChassisSpeeds defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public ChassisSpeeds[] getOutput(String key, ChassisSpeeds[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Clears all publishers
     */
    public void clearPublishers(){
        registeredPublishers.clear();
    }

    @Override
    public final void periodic() {
        for (Runnable publisher : registeredPublishers) publisher.run();

        NetworkPeriodic();
    }

    /**
     * The main subsystem loop, replacing periodic() function. All periodic updates should be done here!
     */
    public abstract void NetworkPeriodic();

  
}
