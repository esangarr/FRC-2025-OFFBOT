package lib.ForgePlus.NetworkTableUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.ForgePlus.Math.Profiles.Control.MotionModelControl;
import lib.ForgePlus.Math.Profiles.Control.PIDControl;

/**
 * Utility class for publishing and retrieving various data types from NetworkTables.
 * This class allows for easy interaction with the NetworkTables API using structured and primitive data types.
 * 
 * <p><b>Supported Data Types:</b></p>
 * <ul>
 *   <li>{@code double}</li>
 *   <li>{@code double[]}</li>
 *   <li>{@code boolean}</li>
 *   <li>{@code boolean[]}</li>
 *   <li>{@code String}</li>
 *   <li>{@code String[]}</li>
 *   <li>{@code ChassisSpeeds}</li>
 *   <li>{@code ChassisSpeeds[]}</li>
 *   <li>{@code Pose2d}</li>
 *   <li>{@code Pose2d[]}</li>
 *   <li>{@code Pose3d}</li>
 *   <li>{@code Pose3d[]}</li>
 *   <li>{@code Rotation2d}</li>
 *   <li>{@code Rotation2d[]}</li>
 *   <li>{@code Rotation3d}</li>
 *   <li>{@code Rotation3d[]}</li>
 *   <li>{@code Transform2d}</li>
 *   <li>{@code Transform2d[]}</li>
 *   <li>{@code Transform3d}</li>
 *   <li>{@code Transform3d[]}</li>
 *   <li>{@code Translation2d}</li>
 *   <li>{@code Translation2d[]}</li>
 *   <li>{@code Translation3d}</li>
 *   <li>{@code Translation3d[]}</li>
 *   <li>{@code SwerveModuleState}</li>
 *   <li>{@code SwerveModuleState[]}</li>
 *   <li>{@code SwerveModulePosition}</li>
 *   <li>{@code SwerveModulePosition[]}</li>
 *   <li>{@code Color}</li>
 *   <li>{@code Sendable}</li>
 *   <li>{@code CommandXboxController}</li>
 *   <li>{@code CommandPS4Controller}</li>
 *   <li>{@code CommandPS5Controller}</li>
 *   <li>{@code PIDControl}</li>
 *   <li>{@code MotionModelControl}</li>
 *   
 * </ul>
 */
public class NTPublisher{

    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
   
    private static final Map<String, Sendable> sendables = new HashMap<>();

    //Common table names

    public static final String SMARTDASHBOARD = "SmartDashboard";

    public static final String NETWORKTABLES = "NetworkTables";

    public static final String ROBOT = "Robot";

    public static final String FORGE = "Forge";

    //Publishers
    private static final ConcurrentHashMap<String, StructPublisher<Translation2d>> t2 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Translation2d>> at2 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Translation3d>> t3 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Translation3d>> at3 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Transform2d>> tf2 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Transform2d>> atf2 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Transform3d>> tf3 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Transform3d>> atf3 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Pose2d>> p2 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArrayPublisher<Pose2d>> ap2 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Pose3d>> p3 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArrayPublisher<Pose3d>> ap3 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Rotation2d>> r2 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Rotation2d>> ar2 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<Rotation3d>> r3 = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<Rotation3d>> ar3 = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<ChassisSpeeds>> cS = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<ChassisSpeeds>> acS = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<SwerveModuleState>> State = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<SwerveModuleState>> aState = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructPublisher<SwerveModulePosition>> Pos = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String,StructArrayPublisher<SwerveModulePosition>> aPos = new ConcurrentHashMap<>();

    //Subscribers
    private static final ConcurrentHashMap<String, StructSubscriber<Translation2d>> t2P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Translation2d>> at2P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Translation3d>> t3P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Translation3d>> at3P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Transform2d>> tf2P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Transform2d>> atf2P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Transform3d>> tf3P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Transform3d>> atf3P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Pose2d>> p2P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Pose2d>> ap2P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Pose3d>> p3P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Pose3d>> ap3P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Rotation2d>> r2P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Rotation2d>> ar2P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<Rotation3d>> r3P = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<Rotation3d>> ar3P = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<ChassisSpeeds>> cSP = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<ChassisSpeeds>> acSP = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<SwerveModuleState>> StateP = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<SwerveModuleState>> aStateP = new ConcurrentHashMap<>();

    private static final ConcurrentHashMap<String, StructSubscriber<SwerveModulePosition>> PosP = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, StructArraySubscriber<SwerveModulePosition>> aPosP = new ConcurrentHashMap<>();

    private NTPublisher(){
        throw new UnsupportedOperationException("This is an Utility class");
    }

    public static String fullPathOf(String table, String key){
        return table + "/" + key;
    }

    private static class JoystickPublisher implements Sendable{

        private DoubleSupplier[] rightAxis = new DoubleSupplier[2];
        private DoubleSupplier[] leftAxis = new DoubleSupplier[2];
        private DoubleSupplier[] triggers = new DoubleSupplier[2];
        private BooleanSupplier[] buttons = new BooleanSupplier[4];
        private BooleanSupplier[] bumpers  = new BooleanSupplier[2];
        private BooleanSupplier[] specialButtons = new BooleanSupplier[2];
        private BooleanSupplier[] stickPressed = new BooleanSupplier[2];
        private BooleanSupplier[] povs = new BooleanSupplier[4];

        private String[] buttonNames = new String[4];
        private String[] specialButtonNames = new String[2];

        private JoystickPublisher() {}

        private JoystickPublisher(String table,String name, CommandXboxController controller){

            this.rightAxis[0] = ()-> controller.getRightY();
            this.rightAxis[1] = ()-> controller.getRightX();

            this.leftAxis[0] = ()-> controller.getLeftY();
            this.leftAxis[1] = ()-> controller.getLeftX();

            this.triggers[0] = ()->controller.getLeftTriggerAxis();
            this.triggers[1] = ()-> controller.getRightTriggerAxis();

            this.buttons[0] = ()-> controller.a().getAsBoolean();
            this.buttons[1] = ()-> controller.b().getAsBoolean();
            this.buttons[2] = ()-> controller.x().getAsBoolean();
            this.buttons[3] = ()-> controller.y().getAsBoolean();

            this.bumpers[0] = ()-> controller.leftBumper().getAsBoolean();
            this.bumpers[1] = ()-> controller.rightBumper().getAsBoolean();

            this.specialButtons[0] = ()-> controller.start().getAsBoolean();
            this.specialButtons[1] = ()-> controller.back().getAsBoolean();

            this.stickPressed[0] = ()-> controller.leftStick().getAsBoolean();
            this.stickPressed[1] = ()-> controller.rightStick().getAsBoolean();

            this.povs[0] = ()-> controller.povDown().getAsBoolean();
            this.povs[1] = ()-> controller.povLeft().getAsBoolean();
            this.povs[2] = ()-> controller.povRight().getAsBoolean();
            this.povs[3] = ()-> controller.povUp().getAsBoolean();

            this.buttonNames[0] = "A";
            this.buttonNames[1] = "B";
            this.buttonNames[2] = "X";
            this.buttonNames[3] = "Y";

            this.specialButtonNames[0] = "Start";
            this.specialButtonNames[1] = "Back";

            publish(table, name, this);

        }

        private JoystickPublisher(String table, String name, CommandPS5Controller controller){

            this.rightAxis[0] = ()-> controller.getRightY();
            this.rightAxis[1] = ()->controller.getRightX();

            this.leftAxis[0] = ()->controller.getLeftY();
            this.leftAxis[1] = ()-> controller.getLeftX();

            this.triggers[0] = ()-> controller.getL2Axis();
            this.triggers[1] = ()-> controller.getR2Axis();

            this.buttons[0] = ()-> controller.cross().getAsBoolean();
            this.buttons[1] = ()-> controller.circle().getAsBoolean();
            this.buttons[2] = ()-> controller.square().getAsBoolean();
            this.buttons[3] = ()-> controller.triangle().getAsBoolean();

            this.bumpers[0] = ()-> controller.L1().getAsBoolean();
            this.bumpers[1] = ()-> controller.R1().getAsBoolean();

            this.specialButtons[0] = ()-> controller.options().getAsBoolean();
            this.specialButtons[1] = ()-> controller.create().getAsBoolean();

            this.stickPressed[0] = ()->controller.L3().getAsBoolean();
            this.stickPressed[1] = ()-> controller.R3().getAsBoolean();

            this.povs[0] = ()-> controller.povDown().getAsBoolean();
            this.povs[1] = ()-> controller.povLeft().getAsBoolean();
            this.povs[2] = ()-> controller.povRight().getAsBoolean();
            this.povs[3] = ()-> controller.povUp().getAsBoolean();

            this.buttonNames[0] = "Cross";
            this.buttonNames[1] = "Circle";
            this.buttonNames[2] = "Square";
            this.buttonNames[3] = "Triangle";

            this.specialButtonNames[0] = "Options";
            this.specialButtonNames[1] = "Create";
        
            publish(table, name, this);

        }

        private JoystickPublisher(String table, String name, CommandPS4Controller controller){

            this.rightAxis[0] = ()-> controller.getRightY();
            this.rightAxis[1] = ()-> controller.getRightX();

            this.leftAxis[0] = ()-> controller.getLeftY();
            this.leftAxis[1] = ()-> controller.getLeftX();

            this.triggers[0] = ()-> controller.getL2Axis();
            this.triggers[1] = ()-> controller.getR2Axis();

            this.buttons[0] = ()-> controller.cross().getAsBoolean();
            this.buttons[1] = ()-> controller.circle().getAsBoolean();
            this.buttons[2] = ()-> controller.square().getAsBoolean();
            this.buttons[3] = ()-> controller.triangle().getAsBoolean();

            this.bumpers[0] = ()-> controller.L1().getAsBoolean();
            this.bumpers[1] = ()-> controller.R1().getAsBoolean();

            this.specialButtons[0] = ()-> controller.options().getAsBoolean();
            this.specialButtons[1] = ()-> controller.share().getAsBoolean();

            this.stickPressed[0] = ()-> controller.L3().getAsBoolean();
            this.stickPressed[1] = ()-> controller.R3().getAsBoolean();

            this.povs[0] = ()-> controller.povDown().getAsBoolean();
            this.povs[1] = ()-> controller.povLeft().getAsBoolean();
            this.povs[2] = ()-> controller.povRight().getAsBoolean();
            this.povs[3] = ()-> controller.povUp().getAsBoolean();

            this.buttonNames[0] = "Cross";
            this.buttonNames[1] = "Circle";
            this.buttonNames[2] = "Square";
            this.buttonNames[3] = "Triangle";

            this.specialButtonNames[0] = "Options";
            this.specialButtonNames[1] = "Share";

            publish(table, name, this);
        }

        @Override
        public void initSendable(SendableBuilder builder){

            builder.setSmartDashboardType("Joystick");

            builder.addDoubleProperty("Axis/Right Axis/Y",  rightAxis[0], null);
            builder.addDoubleProperty("Axis/Right Axis/X",  rightAxis[1], null);

            builder.addDoubleProperty("Axis/Left Axis/Y",  leftAxis[0], null);
            builder.addDoubleProperty("Axis/Left Axis/X",  leftAxis[1], null);

            builder.addDoubleProperty("Triggers/Left",  triggers[0], null);
            builder.addDoubleProperty("Triggers/Right",  triggers[1], null);

            builder.addBooleanProperty("Buttons/" + buttonNames[0],  buttons[0], null);
            builder.addBooleanProperty("Buttons/" + buttonNames[1],  buttons[1], null);
            builder.addBooleanProperty("Buttons/" + buttonNames[2],  buttons[2], null);
            builder.addBooleanProperty("Buttons/" + buttonNames[3],  buttons[3], null);

            builder.addBooleanProperty("Bumpers/Left",  bumpers[0], null);
            builder.addBooleanProperty("Bumpers/Right",  bumpers[1], null);

            builder.addBooleanProperty("Special Buttons/" + specialButtonNames[0],  specialButtons[0], null);
            builder.addBooleanProperty("Special Buttons/" + specialButtonNames[1],  specialButtons[1], null);

            builder.addBooleanProperty("Axis/Stick Pressed/Left",  stickPressed[0], null);
            builder.addBooleanProperty("Axis/Stick Pressed/Right",  stickPressed[1], null);

            builder.addBooleanProperty("POVs/Down",  povs[0], null);
            builder.addBooleanProperty("POVs/Left",  povs[1], null);
            builder.addBooleanProperty("POVs/Right",  povs[2], null);
            builder.addBooleanProperty("POVs/Up",  povs[3], null);

        }

    }

    /**
     * Publishes a double value to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The double value to publish.
     */
    public static void publish(String tableName, String key, double value){
        ntInstance.getTable(tableName).getEntry(key).setDouble(value);
    }

    /**
     * Publishes a boolean value to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The boolean value to publish.
     */
    public static void publish(String tableName, String key, boolean value){
        ntInstance.getTable(tableName).getEntry(key).setBoolean(value);
    }

    /**
     * Publishes a String value to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The String value to publish.
     */
    public static void publish(String tableName, String key, String value){
        ntInstance.getTable(tableName).getEntry(key).setString(value);
    }

    /**
     * Publishes an array of doubles to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The double array to publish.
     */
    public static void publish(String tableName, String key, double[] value){
        ntInstance.getTable(tableName).getEntry(key).setDoubleArray(value);
    }

    /**
     * Publishes an array of booleans to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The boolean array to publish.
     */
    public static void publish(String tableName, String key, boolean[] value){
        ntInstance.getTable(tableName).getEntry(key).setBooleanArray(value);
    }

    /**
     * Publishes an array of Strings to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The String array to publish.
     */
    public static void publish(String tableName, String key, String[] value){
        ntInstance.getTable(tableName).getEntry(key).setStringArray(value);
    }

    /**
     * Publishes a {@link ChassisSpeeds} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link ChassisSpeeds} object to publish.
     */
    public static void publish(String tableName, String key, ChassisSpeeds value){
        
        StructPublisher<ChassisSpeeds> subscriber = cS.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, ChassisSpeeds.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link ChassisSpeeds} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link ChassisSpeeds} to publish.
     */
    public static void publish(String tableName, String key, ChassisSpeeds[] value){
        
        StructArrayPublisher<ChassisSpeeds> subscriber = acS.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, ChassisSpeeds.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link Pose2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Pose2d} object to publish.
     */
    public static void publish(String tableName, String key, Pose2d value){
        
        StructPublisher<Pose2d> subscriber = p2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Pose2d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link Pose2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Pose2d} to publish.
     */
    public static void publish(String tableName, String key, Pose2d[] value){

        StructArrayPublisher<Pose2d> subscriber = ap2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Pose2d.struct).publish()
        );

        subscriber.set(value);
    }


    /**
     * Publishes a {@link Pose3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Pose3d} object to publish.
     */
    public static void publish(String tableName, String key, Pose3d value){
        
        StructPublisher<Pose3d> subscriber = p3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Pose3d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes an array of {@link Pose3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Pose3d} to publish.
     */
    public static void publish(String tableName, String key, Pose3d[] value){
        
        StructArrayPublisher<Pose3d> subscriber = ap3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Pose3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link Rotation2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Rotation2d} object to publish.
     */
    public static void publish(String tableName, String key, Rotation2d value){

        StructPublisher<Rotation2d> subscriber = r2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Rotation2d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes an array of {@link Rotation2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Rotation2d} to publish.
     */
    public static void publish(String tableName, String key, Rotation2d[] value){

        StructArrayPublisher<Rotation2d> subscriber = ar2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Rotation2d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes a {@link Rotation3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Rotation3d} object to publish.
     */
    public static void publish(String tableName, String key, Rotation3d value){
        
        StructPublisher<Rotation3d> subscriber = r3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Rotation3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link Rotation3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Rotation3d} to publish.
     */
    public static void publish(String tableName, String key, Rotation3d[] value){
        
        StructArrayPublisher<Rotation3d> subscriber = ar3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Rotation3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link Transform2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Transform2d} object to publish.
     */
    public static void publish(String tableName, String key, Transform2d value){
        
        StructPublisher<Transform2d> subscriber = tf2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Transform2d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes an array of {@link Transform2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Transform2d} to publish.
     */
    public static void publish(String tableName, String key, Transform2d[] value){
        
        StructArrayPublisher<Transform2d> subscriber = atf2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Transform2d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes a {@link Transform3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Transform3d} object to publish.
     */
    public static void publish(String tableName, String key, Transform3d value){
        
        StructPublisher<Transform3d> subscriber = tf3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Transform3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link Transform3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Transform3d} to publish.
     */
    public static void publish(String tableName, String key, Transform3d[] value){
        
        StructArrayPublisher<Transform3d> subscriber = atf3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Transform3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link Translation2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Translation2d} object to publish.
     */
    public static void publish(String tableName, String key, Translation2d value){
        
        StructPublisher<Translation2d> subscriber = t2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Translation2d.struct).publish()
        );

        subscriber.set(value);
    }

    /**
     * Publishes an array of {@link Translation2d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Translation2d} to publish.
     */
    public static void publish(String tableName, String key, Translation2d[] value){
        
        StructArrayPublisher<Translation2d> subscriber = at2.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Translation2d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link Translation3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Translation3d} object to publish.
     */
    public static void publish(String tableName, String key, Translation3d value){
        
        StructPublisher<Translation3d> subscriber = t3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, Translation3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link Translation3d} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Translation3d} to publish.
     */
    public static void publish(String tableName, String key, Translation3d[] value){
        
        StructArrayPublisher<Translation3d> subscriber = at3.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, Translation3d.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes a {@link SwerveModuleState} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link SwerveModuleState} object to publish.
     */
    public static void publish(String tableName, String key, SwerveModuleState value){
        
        StructPublisher<SwerveModuleState> subscriber = State.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, SwerveModuleState.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link SwerveModuleState} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link SwerveModuleState} to publish.
     */
    public static void publish(String tableName, String key, SwerveModuleState[] value){

        StructArrayPublisher<SwerveModuleState> subscriber = aState.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, SwerveModuleState.struct).publish()
        );

        subscriber.set(value);       
    }

    /**
     * Publishes a {@link SwerveModulePosition} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link SwerveModulePosition} object to publish.
     */
    public static void publish(String tableName, String key, SwerveModulePosition value){
        
        StructPublisher<SwerveModulePosition> subscriber = Pos.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructTopic(key, SwerveModulePosition.struct).publish()
        );

        subscriber.set(value);

    }

    /**
     * Publishes an array of {@link SwerveModulePosition} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link SwerveModulePosition} to publish.
     */
    public static void publish(String tableName, String key, SwerveModulePosition[] value){
        
        StructArrayPublisher<SwerveModulePosition> subscriber = aPos.computeIfAbsent(fullPathOf(tableName, key), k ->
            ntInstance.getTable(tableName).getStructArrayTopic(key, SwerveModulePosition.struct).publish()
        );

        subscriber.set(value); 

    }

    /**
     * Publishes a {@link Sendable} object to NetworkTables.
     * Ensures that the sendable is only added once per unique table entry.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Sendable} object to publish.
     */
    public static void publish(String tableName, String key, Sendable value) {
        String fullPath = String.format("%s/%s", tableName, key);
        if (!sendables.containsKey(fullPath) || sendables.get(fullPath) != value) {
            sendables.put(fullPath, value);
            NetworkTable dataTable = NetworkTableInstance.getDefault().getTable(fullPath);
            SendableBuilderImpl builder = new SendableBuilderImpl();
            builder.setTable(dataTable);
            SendableRegistry.publish(value, builder);
            builder.startListeners();
            dataTable.getEntry(".name").setString(fullPath);
        }
    }

    /**
     * Publishes a {@link Color} value as a hex string to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link Color} object to publish.
     */
    public static void publish(String tableName, String key, Color value){
        ntInstance.getTable(tableName).getEntry(key).setString(value.toHexString());
    }

    /**
     * Publishes a controller to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The controller to publish.
     */
    public static void publish(String tableName,String key, CommandXboxController value){
        new JoystickPublisher(tableName, key, value);
    }

    /**
     * Publishes a controller to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The controller to publish.
     */
    public static void publish(String tableName,String key,  CommandPS5Controller value){
        new JoystickPublisher(tableName, key, value);
    }

    /**
     * Publishes a controller to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The controller to publish.
     */
    public static void publish(String tableName,String key, CommandPS4Controller value){
        new JoystickPublisher(tableName, key, value);
    }

    /**
     * Publishes a {@link MotionModelControl} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link MotionModelControl} object to publish.
     */
    public static void publish(String tableName, String key, MotionModelControl value){
        publish(tableName, key, value.getController());
    }

    /**
     * Publishes a {@link PIDControl} object to NetworkTables.
     *
     * @param tableName The name of the table.
     * @param key       The key for the entry.
     * @param value     The {@link PIDControl} object to publish.
     */
    public static void publish(String tableName, String key, PIDControl value){
        publish(tableName, key, value.getController());
    }

    /**
     * Updates all registered {@link Sendable} objects in NetworkTables. <b>Must call this method periodically</b>
     */
    public static void updateAllSendables() {
        for (Sendable sendable : sendables.values()) {
            SendableRegistry.update(sendable);
        }
    }

    /**
     * Retrieves a double value from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved double value.
     */
    public static double retrieve(String TableName, String key, double defaultValue){

        return ntInstance.getTable(TableName).getEntry(key).getDouble(defaultValue);
    }

    /**
     * Retrieves a boolean value from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved boolean value.
     */
    public static boolean retrieve(String TableName, String key, boolean defaultValue){
        return ntInstance.getTable(TableName).getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Retrieves an array of double values from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of double values.
     */
    public static double[] retrieve(String TableName, String key, double[] defaultValue){
        return ntInstance.getTable(TableName).getEntry(key).getDoubleArray(defaultValue);
    }

    /**
     * Retrieves an array of boolean values from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of boolean values.
     */
    public static boolean[] retrieve(String TableName, String key, boolean[] defaultValue){
        return ntInstance.getTable(TableName).getEntry(key).getBooleanArray(defaultValue);
    }

    /**
     * Retrieves a String value from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved String value.
     */
    public static String retrieve(String TableName, String key, String defaultValue){
        return ntInstance.getTable(TableName).getEntry(key).getString(defaultValue);
    }

    /**
     * Retrieves an array of String values from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of String values.
     */
    public static String[] retrieve(String TableName, String key, String[] defaultValue){
        return ntInstance.getTable(TableName).getEntry(key).getStringArray(defaultValue);
    }

    /**
     * Retrieves a {@link Pose2d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Pose2d} object.
     */
    public static Pose2d retrieve(String TableName, String key, Pose2d defaultValue){

        StructSubscriber<Pose2d> subscriber = p2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Pose2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Pose2d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Pose2d} objects.
     */
    public static Pose2d[] retrieve(String TableName, String key, Pose2d[] defaultValue){

        StructArraySubscriber<Pose2d> subscriber = ap2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Pose2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Pose3d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Pose3d} object.
     */
    public static Pose3d retrieve(String TableName, String key, Pose3d defaultValue){
    
        StructSubscriber<Pose3d> subscriber = p3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Pose3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Pose3d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Pose3d} objects.
     */
    public static Pose3d[] retrieve(String TableName, String key, Pose3d[] defaultValue){

        StructArraySubscriber<Pose3d> subscriber = ap3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Pose3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Translation2d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Translation2d} object.
     */
    public static Translation2d retrieve(String TableName, String key, Translation2d defaultValue){

        StructSubscriber<Translation2d> subscriber = t2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Translation2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Translation2d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Translation2d} objects.
     */
    public static Translation2d[] retrieve(String TableName, String key, Translation2d[] defaultValue){
        
        StructArraySubscriber<Translation2d> subscriber = at2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Translation2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Translation3d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Translation3d} object.
     */
    public static Translation3d retrieve(String TableName, String key, Translation3d defaultValue){

        StructSubscriber<Translation3d> subscriber = t3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Translation3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Translation3d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Translation3d} objects.
     */
    public static Translation3d[] retrieve(String TableName, String key, Translation3d[] defaultValue){

        StructArraySubscriber<Translation3d> subscriber = at3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Translation3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Transform2d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Transform2d} object.
     */
    public static Transform2d retrieve(String TableName, String key, Transform2d defaultValue){

        StructSubscriber<Transform2d> subscriber = tf2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Transform2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Transform2d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Transform2d} objects.
     */
    public static Transform2d[] retrieve(String TableName, String key, Transform2d[] defaultValue){
        
        StructArraySubscriber<Transform2d> subscriber = atf2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Transform2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Transform3d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Transform3d} object.
     */
    public static Transform3d retrieve(String TableName, String key, Transform3d defaultValue){
        
        StructSubscriber<Transform3d> subscriber = tf3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Transform3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Transform3d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Transform3d} objects.
     */
    public static Transform3d[] retrieve(String TableName, String key, Transform3d[] defaultValue){
        
        StructArraySubscriber<Transform3d> subscriber = atf3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Transform3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Rotation2d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Rotation2d} object.
     */
    public static Rotation2d retrieve(String TableName, String key, Rotation2d defaultValue){

        StructSubscriber<Rotation2d> subscriber = r2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Rotation2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link Rotation2d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Rotation2d} objects.
     */
    public static Rotation2d[] retrieve(String TableName, String key, Rotation2d[] defaultValue){

        StructArraySubscriber<Rotation2d> subscriber = ar2P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Rotation2d.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link Rotation3d} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link Rotation3d} object.
     */
    public static Rotation3d retrieve(String TableName, String key, Rotation3d defaultValue){
        
        StructSubscriber<Rotation3d> subscriber = r3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, Rotation3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();

    }

    /**
     * Retrieves an array of {@link Rotation3d} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link Rotation3d} objects.
     */
    public static Rotation3d[] retrieve(String TableName, String key, Rotation3d[] defaultValue){
        
        StructArraySubscriber<Rotation3d> subscriber = ar3P.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, Rotation3d.struct).subscribe(defaultValue)
        );

        return subscriber.get();

    }

    /**
     * Retrieves a {@link ChassisSpeeds} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link ChassisSpeeds} object.
     */
    public static ChassisSpeeds retrieve(String TableName, String key, ChassisSpeeds defaultValue){

        StructSubscriber<ChassisSpeeds> subscriber = cSP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, ChassisSpeeds.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link ChassisSpeeds} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link ChassisSpeeds} objects.
     */
    public static ChassisSpeeds[] retrieve(String TableName, String key, ChassisSpeeds[] defaultValue){

        StructArraySubscriber<ChassisSpeeds> subscriber = acSP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, ChassisSpeeds.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves a {@link SwerveModuleState} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link SwerveModuleState} object.
     */
    public static SwerveModuleState retrieve(String TableName, String key, SwerveModuleState defaultValue){

        StructSubscriber<SwerveModuleState> subscriber = StateP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, SwerveModuleState.struct).subscribe(defaultValue)
        );

        return subscriber.get();

    }

    /**
     * Retrieves an array of {@link SwerveModuleState} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link SwerveModuleState} objects.
     */
    public static SwerveModuleState[] retrieve(String TableName, String key, SwerveModuleState[] defaultValue){
        
        StructArraySubscriber<SwerveModuleState> subscriber = aStateP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, SwerveModuleState.struct).subscribe(defaultValue)
        );

        return subscriber.get();

    }

    /**
     * Retrieves a {@link SwerveModulePosition} object from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default value if the key is not found.
     * @return The retrieved {@link SwerveModulePosition} object.
     */
    public static SwerveModulePosition retrieve(String TableName, String key, SwerveModulePosition defaultValue){

        StructSubscriber<SwerveModulePosition> subscriber = PosP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructTopic(key, SwerveModulePosition.struct).subscribe(defaultValue)
        );

        return subscriber.get();
    }

    /**
     * Retrieves an array of {@link SwerveModulePosition} objects from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default array if the key is not found.
     * @return The retrieved array of {@link SwerveModulePosition} objects.
     */
    public static SwerveModulePosition[] retrieve(String TableName, String key, SwerveModulePosition[] defaultValue){
        
        StructArraySubscriber<SwerveModulePosition> subscriber = aPosP.computeIfAbsent(fullPathOf(TableName, key), k ->
            ntInstance.getTable(TableName).getStructArrayTopic(key, SwerveModulePosition.struct).subscribe(defaultValue)
        );

        return subscriber.get();

    }

    /**
     * Retrieves a {@link Color} value from NetworkTables.
     *
     * @param tableName    The name of the table.
     * @param key          The key for the entry.
     * @param defaultValue The default {@link Color} if the key is not found.
     * @return The retrieved {@link Color} object.
     */
    public static Color retrieve(String TableName, String key, Color defaultValue){
        return new Color(ntInstance.getTable(TableName).getEntry(key).getString(defaultValue.toHexString()));
    } 

}
