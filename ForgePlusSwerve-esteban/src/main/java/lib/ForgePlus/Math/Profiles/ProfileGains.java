package lib.ForgePlus.Math.Profiles;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ProfileGains{

    private ProfileGains(){}

    public interface Gains extends Sendable{

        public default double[] toArray(){return new double[]{};};

        public default Gains fromArray(double[] array){return new Gains() {};}

        @Override
        public default void initSendable(SendableBuilder builder) {}
     
    }

    public record PIDGains(double kP, double kI, double kD) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kP,kI,kD};
        }

        @Override
        public Gains fromArray(double[] array){
            return new PIDGains(array[0], array[1], array[2]);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: P", () -> kP, null);
            builder.addDoubleProperty("slot1: I", () -> kI, null);
            builder.addDoubleProperty("slot2: D", () -> kD, null);

            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }

    public record PIDFGains(double kP, double kI, double kD, double kS, double kV) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kP,kI,kD,kS,kV};
        }

        @Override
        public Gains fromArray(double[] array){
            return new PIDFGains(array[0], array[1], array[2], array[3], array[4]);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: P", () -> kP, null);
            builder.addDoubleProperty("slot1: I", () -> kI, null);
            builder.addDoubleProperty("slot2: D", () -> kD, null);
            builder.addDoubleProperty("slot3: S", () -> kS, null);
            builder.addDoubleProperty("slot4: V", () -> kV, null);

            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }

    public record SimpleFeedForwardGains(double kS, double kV, double kA) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kS, kV, kA};
        }

        @Override
        public Gains fromArray(double[]array){
            return new SimpleFeedForwardGains(array[0], array[1], array[2]);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: S", () -> kS, null);
            builder.addDoubleProperty("slot1: V", () -> kV, null);
            builder.addDoubleProperty("slot2: A", () -> kA, null);
     
            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }

    public record CompleteFeedForwardGains(double kS, double kV, double kA, double kG) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kS,kV,kA,kG};
        }

        @Override
        public Gains fromArray(double[] array){
            return new CompleteFeedForwardGains(array[0], array[1], array[2], array[3]);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: S", () -> kS, null);
            builder.addDoubleProperty("slot1: V", () -> kV, null);
            builder.addDoubleProperty("slot2: A", () -> kA, null);
            builder.addDoubleProperty("slot3: G", () -> kG, null);
     
            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }

    public record MotionModelGains(
        double kP, double kI, double kD,
        double maxAcceleration, double maxVelocity) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kP, kI,kD,maxAcceleration, maxVelocity};
        }

        @Override
        public MotionModelGains fromArray(double[] array){
            return new MotionModelGains(
                array[0], array[1], array[2], array[3], array[4]
            );
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: P", () -> kP, null);
            builder.addDoubleProperty("slot1: I", () -> kI, null);
            builder.addDoubleProperty("slot2: D", () -> kD, null);

            builder.addDoubleProperty("slot7: maxAcceleration", () -> maxAcceleration, null);
            builder.addDoubleProperty("slot8: maxVelocity", () -> maxVelocity, null);

            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }

    public record MotionModelGainsExpo(
        double kP, double kI, double kD,
        double maxAcceleration, double maxVelocity, double jerk) implements Gains{

        @Override
        public double[] toArray(){
            return new double[]{kP, kI,kD,maxAcceleration, maxVelocity, jerk};
        }

        @Override
        public MotionModelGainsExpo fromArray(double[] array){
            return new MotionModelGainsExpo(
                array[0], array[1], array[2], array[3], array[4], array[5]
            );
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("ProfileGains");

            builder.addDoubleProperty("slot0: P", () -> kP, null);
            builder.addDoubleProperty("slot1: I", () -> kI, null);
            builder.addDoubleProperty("slot2: D", () -> kD, null);

            builder.addDoubleProperty("slot7: maxAcceleration", () -> maxAcceleration, null);
            builder.addDoubleProperty("slot8: maxVelocity", () -> maxVelocity, null);

            builder.addDoubleProperty("slot9: jerk", () -> jerk, null);

            builder.addStringProperty("Gains", () -> getClass().getName(), null);
            builder.addStringProperty("GainsList", () -> java.util.Arrays.toString(toArray()), null);
        }
    }
    
}

