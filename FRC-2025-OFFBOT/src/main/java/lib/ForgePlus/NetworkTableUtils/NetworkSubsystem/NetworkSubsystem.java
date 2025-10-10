package lib.ForgePlus.NetworkTableUtils.NetworkSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ForgePlus.NetworkTableUtils.NTPublisher;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.util.Color;

import java.lang.reflect.Method;
import java.util.*;
import java.util.function.Supplier;

public abstract class NetworkSubsystem extends SubsystemBase{

    private final Set<Runnable> registeredPublishers = new HashSet<>();
    private final String tableName;

    public static final double kDefaultPeriod = 0.02;

    /**
     * Creates a NetworkSubsystem with the specified table name and subsystem info publishing option.
     * @param tableName the name of the NetworkTables table to use for this subsystem
     * @param subsystemInfo if true, publishes the entire subsystem object to NetworkTables
     */
    public NetworkSubsystem(String tableName, boolean subsystemInfo) {
        this.tableName = tableName;
        clearPublishers();

        NTPublisher.publish(tableName, "TableKey", getTableKey());
    
        if (subsystemInfo) {
            //NTPublisher.publish(tableName, "Subsystem", this);
        }

        registerAnnotatedPublishers(tableName);
        
    }

    /**
     * Clears all publishers
     */
    public void clearPublishers(){
        System.out.println("[NetworkSubsystem] Clearing publishers for " + tableName);
        registeredPublishers.clear();
        NTPublisher.clearForTable(tableName);
    }

    /**
     * Gets the subsystem Key in NetworkTables
     * @return the key
     */
    public String getTableKey(){
        return tableName;
    }
    
    private void registerAnnotatedPublishers(String tableName) {

        System.out.println("[NetworkSubsystem] Registering publishers for " + tableName);
    
        if (!registeredPublishers.isEmpty()) {
            System.err.println("[NetworkSubsystem] WARNING: registerAnnotatedPublishers() Overloop!");
            return;
        }
    
        int count = 0;
    
        for (Method method : this.getClass().getDeclaredMethods()) {

            boolean isPeriodic = method.isAnnotationPresent(AutoNetworkPublish.class);
            boolean isOnce = method.isAnnotationPresent(AutoStaticPublish.class);

            if (!isPeriodic && !isOnce) continue;
    
            String key = (isPeriodic
                ? method.getAnnotation(AutoNetworkPublish.class).key()
                : method.getAnnotation(AutoStaticPublish.class).key());
    
            method.setAccessible(true);
    
            Supplier<?> supplier = () -> {
                try {
                    return method.invoke(this);
                } catch (Exception e) {
                    e.printStackTrace();
                    return null;
                }
            };

            Class<?> returnType = method.getReturnType();

            if (isPeriodic) {
                try {
                    registerPublisher(() -> publishByType(tableName, key, supplier, method.getReturnType()));
                } catch (Exception e) {
                    System.err.println("[AutoNetworkPublisher] Error registering key " + key + " in table " + tableName);
                    e.printStackTrace();
                }
            }else{
                try {
                    publishByType(tableName, key, supplier, method.getReturnType());
                } catch (Exception e) {
                    System.err.println("[AutoNetworkPublisher] Error registering key " + key + " in table " + tableName);
                    e.printStackTrace();
                }
            }
        }

        count++;
    
        System.out.println("[NetworkSubsystem] Total publishers registered for " + tableName + ": " + count);
    }

 
    private void publishByType(String tableName, String key, Supplier<?> supplier, Class<?> returnType) {

        try {
            if (returnType == Pose2d.class) {
                NTPublisher.publish(tableName, key, (Pose2d) supplier.get());
    
            } else if (returnType == Pose2d[].class) {
                NTPublisher.publish(tableName, key, (Pose2d[]) supplier.get());
    
            } else if (returnType == Pose3d.class) {
                NTPublisher.publish(tableName, key, (Pose3d) supplier.get());
    
            } else if (returnType == Pose3d[].class) {
                NTPublisher.publish(tableName, key, (Pose3d[]) supplier.get());
    
            } else if (returnType == Rotation2d.class) {
                NTPublisher.publish(tableName, key, (Rotation2d) supplier.get());
    
            } else if (returnType == Rotation2d[].class) {
                NTPublisher.publish(tableName, key, (Rotation2d[]) supplier.get());
    
            } else if (returnType == Rotation3d.class) {
                NTPublisher.publish(tableName, key, (Rotation3d) supplier.get());
    
            } else if (returnType == Translation2d.class) {
                NTPublisher.publish(tableName, key, (Translation2d) supplier.get());
    
            } else if (returnType == Translation2d[].class) {
                NTPublisher.publish(tableName, key, (Translation2d[]) supplier.get());
    
            } else if (returnType == Translation3d.class) {
                NTPublisher.publish(tableName, key, (Translation3d) supplier.get());
    
            } else if (returnType == Translation3d[].class) {
                NTPublisher.publish(tableName, key, (Translation3d[]) supplier.get());
    
            } else if (returnType == ChassisSpeeds.class) {
                NTPublisher.publish(tableName, key, (ChassisSpeeds) supplier.get());
    
            } else if (returnType == SwerveModuleState[].class) {
                NTPublisher.publish(tableName, key, (SwerveModuleState[]) supplier.get());
    
            } else if (returnType == SwerveModulePosition[].class) {
                NTPublisher.publish(tableName, key, (SwerveModulePosition[]) supplier.get());
    
            } else if (returnType == double[].class) {
                NTPublisher.publish(tableName, key, (double[]) supplier.get());
    
            } else if (returnType == Double[].class) {
                Double[] arr = (Double[]) supplier.get();
                if (arr != null) {
                    double[] prim = new double[arr.length];
                    for (int i = 0; i < arr.length; i++) prim[i] = arr[i];
                    NTPublisher.publish(tableName, key, prim);
                }
    
            } else if (returnType == double.class || returnType == Double.class) {
                NTPublisher.publish(tableName, key, ((Number) supplier.get()).doubleValue());
    
            } else if (returnType == boolean.class || returnType == Boolean.class) {
                NTPublisher.publish(tableName, key, (boolean) supplier.get());
    
            } else if (returnType == boolean[].class) {
                NTPublisher.publish(tableName, key, (boolean[]) supplier.get());
    
            } else if (returnType == Boolean[].class) {
                Boolean[] arr = (Boolean[]) supplier.get();
                if (arr != null) {
                    boolean[] prim = new boolean[arr.length];
                    for (int i = 0; i < arr.length; i++) prim[i] = arr[i];
                    NTPublisher.publish(tableName, key, prim);
                }
    
            } else {
                System.err.println("[AutoNetworkPublisher] Not Supported Data type! " + returnType.getSimpleName());
            }
    
        } catch (Exception e) {
            System.err.println("[AutoNetworkPublisher] Error publishing key " + key + " in table " + tableName);
            e.printStackTrace();
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
    public final void publish(String key, double value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, double[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, boolean value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, boolean[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, String value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, String[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Color value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Pose2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Pose2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Pose3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Pose3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Rotation2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Rotation2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Rotation3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Rotation3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Translation2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Translation2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Translation3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Translation3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Transform2d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
    * Publishes an object to NetworkTables.
    * @param key the subkey for showing on NT
    * @param value the object to publish
    */
    public final void publish(String key, Transform2d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Transform3d value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Transform3d[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, SwerveModulePosition value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, SwerveModulePosition[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, SwerveModuleState value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, SwerveModuleState[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, ChassisSpeeds value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, ChassisSpeeds[] value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Publishes an object to NetworkTables.
     * @param key the subkey for showing on NT
     * @param value the object to publish
     */
    public final void publish(String key, Sendable value){
        NTPublisher.publish(getTableKey(), key, value);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public double retrieve(String key, double defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public double[] retrieve(String key, double[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public boolean retrieve(String key, boolean defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public boolean[] retrieve(String key, boolean[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public String retrieve(String key, String defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public String[] retrieve(String key, String[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Color retrieve(String key, Color defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose2d retrieve(String key, Pose2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose2d[] retrieve(String key, Pose2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose3d retrieve(String key, Pose3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Pose3d[] retrieve(String key, Pose3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation2d retrieve(String key, Rotation2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation2d[] retrieve(String key, Rotation2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation3d retrieve(String key, Rotation3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Rotation3d[] retrieve(String key, Rotation3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation2d retrieve(String key, Translation2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation2d[] retrieve(String key, Translation2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation3d retrieve(String key, Translation3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Translation3d[] retrieve(String key, Translation3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform2d retrieve(String key, Transform2d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform2d[] retrieve(String key, Transform2d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform3d retrieve(String key, Transform3d defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public Transform3d[] retrieve(String key, Transform3d[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModulePosition retrieve(String key, SwerveModulePosition defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModulePosition[] retrieve(String key, SwerveModulePosition[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModuleState retrieve(String key, SwerveModuleState defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public SwerveModuleState[] retrieve(String key, SwerveModuleState[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public ChassisSpeeds retrieve(String key, ChassisSpeeds defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    /**
     * Get an object from NetworkTables.
     * @param key the subkey on NT
     * @param defaultValue the defaultValue if the NT object doesn't exist
     * @return value of the object
     */
    public ChassisSpeeds[] retrieve(String key, ChassisSpeeds[] defaultValue){
        return NTPublisher.retrieve(getTableKey(), key, defaultValue);
    }

    @Override
    public void periodic() {
        for (Runnable publisher : registeredPublishers) {
            try {
                publisher.run();
            } catch (Exception e) {
                System.err.println("[NetworkSubsystem] Publisher failed: " + e.getMessage());
                e.printStackTrace();
            }
        }
        NetworkPeriodic();
    }

    /**
     * The main subsystem loop, replacing periodic() function. All periodic updates should be done here!
     */
    public abstract void NetworkPeriodic();

  
}
