package lib.ForgePlus.Math;

import java.util.function.BooleanSupplier;

public class BooleanTrigger{

    private final BooleanSupplier condition;
    private boolean lastState = false;

    public BooleanTrigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    private boolean get() {
        return condition.getAsBoolean();
    }

    public boolean onTrue() {
        boolean currentState = get();
        boolean result = !lastState && currentState;
        lastState = currentState;
        return result;
    }

    public boolean onFalse() {
        boolean currentState = get();
        boolean result = lastState && !currentState;
        lastState = currentState;
        return result;
    }

    public boolean onChange(){
        return onTrue() || onFalse();
    }

    public BooleanTrigger and(BooleanTrigger other) {
        return new BooleanTrigger(() -> this.get() && other.get());
    }

    public BooleanTrigger or(BooleanTrigger other) {
        return new BooleanTrigger(() -> this.get() || other.get());
    }

    public BooleanTrigger not() {
        return new BooleanTrigger(() -> !this.get());
    }

    public boolean whileTrue() {
        return get();
    }

    public boolean whileFalse() {
        return !get();
    }

}
