package lib.ForgePlus.Math.Profiles;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lib.ForgePlus.Math.Profiles.ProfileGains.CompleteFeedForwardGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.MotionModelGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.PIDGains;
import lib.ForgePlus.Math.Profiles.ProfileGains.SimpleFeedForwardGains;

public class Control {
 
    @FunctionalInterface
    public interface ControlResult{
        double getOutput();

        default ControlResult plus(ControlResult other){
            return ()-> getOutput() + other.getOutput();
        }

        default ControlResult minus(ControlResult other){
            return ()-> getOutput() - other.getOutput();
        }

        default ControlResult clamp(double min, double max){
            return ()-> Math.max(Math.min(getOutput(), max), min);
        }

        default ControlResult negate(){
            return ()-> -getOutput();
        }

        default ControlResult times(double scalar){
            return ()-> getOutput() * scalar;
        }

        default ControlResult divide(double scalar){
            return ()-> getOutput() / scalar;
        }

        default ControlResult withDeadband(double threshold) {
            return () -> Math.abs(getOutput()) > threshold ? getOutput() : 0.0;
        }

        default ControlResult delete(){
            return ()-> 0.0;
        }

    }

    public static abstract class ControlBase<S, M>{

        public abstract ControlResult calculate(S setpoint, M measurement);

        public abstract ControlResult calculate(M measurement);

        public abstract S getSetpoint();

        public abstract void setSetpoint(S setpoint);
    
    }

    public static class FeedForwardControl{

        public static ControlResult calculate(SimpleFeedForwardGains gains, double velocity, double acceleration){
            return ()-> 
            Math.signum(velocity) * gains.kS() +
            gains.kV() * velocity +
            gains.kA() * acceleration;
        }

        public static ControlResult calculate(CompleteFeedForwardGains gains, double velocity, double acceleration){
            return ()-> 
            Math.signum(velocity) * gains.kS() +
            gains.kG() +
            gains.kV() * velocity +
            gains.kA() * acceleration;
        }

    }

    public static class PIDControl extends Control.ControlBase<Double, Double>{

        private PIDGains gains;
        private final PIDController controller;

        public PIDControl(PIDGains gains){
            this.gains = gains;
            controller = new PIDController(gains.kP(), gains.kI(), gains.kD());

        }

        public PIDControl(PIDGains gains, double period){
            this.gains = gains;
            controller = new PIDController(gains.kP(), gains.kI(), gains.kD(), period);
        }

        public void setGains(PIDGains gains){
            this.gains = gains;
            controller.setP(gains.kP());
            controller.setI(gains.kI());
            controller.setD(gains.kD());
        }

        public PIDGains getGains(){
            return gains;
        }

        @Override
        public ControlResult calculate(Double setpoint, Double measurement){
            return ()-> controller.calculate(setpoint, measurement);
        }

        @Override
        public ControlResult calculate(Double measurement){
            return ()-> controller.calculate(measurement);
        }

        @Override
        public Double getSetpoint(){
            return controller.getSetpoint();
        }

        @Override
        public void setSetpoint(Double setpoint){
            controller.setSetpoint(setpoint);
        }

        public void disableContinuousInput(){
            controller.disableContinuousInput();
        }

        public void continuousInput(double minInput, double maxInput){
            controller.enableContinuousInput(minInput, maxInput);
        }

        public void setTolerance(double tolerance){
            controller.setTolerance(tolerance);
        }

        public PIDController getController(){
            return controller;
        }

        public void reset(){
            controller.reset();
        }

    }

    public static class PositionState extends TrapezoidProfile.State {

        public PositionState(double position) {
            super(position, 0);
        }
        
    }

    public static class MotionModelControl extends Control.ControlBase<TrapezoidProfile.State, Double>{

        private final ProfiledPIDController controller;
        private MotionModelGains gains;

        public MotionModelControl(MotionModelGains gains) {

            this.gains = gains;
        
            this.controller = new ProfiledPIDController(
                gains.kP(), gains.kI(), gains.kD(),
                new TrapezoidProfile.Constraints(
                    gains.maxVelocity(), gains.maxAcceleration()));
        }

        public void setGains(MotionModelGains gains){
            controller.setP(gains.kP());
            controller.setI(gains.kI());
            controller.setD(gains.kD());
            controller.setConstraints(new TrapezoidProfile.Constraints(gains.maxVelocity(), gains.maxAcceleration()));
        }

        public MotionModelGains getGains(){
            return gains;
        }

        @Override
        public ControlResult calculate(TrapezoidProfile.State setpoint, Double measurement) {
            return ()-> controller.calculate(measurement, setpoint);
        }

        @Override
        public ControlResult calculate(Double measurement) {
            return ()-> controller.calculate(measurement);
        }

        @Override
        public TrapezoidProfile.State getSetpoint() {
            return controller.getSetpoint();
        }

        @Override
        public void setSetpoint(TrapezoidProfile.State setpoint) {
            controller.setGoal(setpoint);
        }

        public void disableContinuousInput(){
            controller.disableContinuousInput();
        }

        public void continuousInput(double minInput, double maxInput){
            controller.enableContinuousInput(minInput, maxInput);
        }

        public void setTolerance(double tolerance, double velocityTolerance){
            controller.setTolerance(tolerance, velocityTolerance);
        }

        public ProfiledPIDController getController() {
            return controller;
        }

        public void reset(double position, double velocity){
            controller.reset(position, velocity);
        }

    }

}
