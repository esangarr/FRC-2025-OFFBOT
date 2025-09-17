package lib.ForgePlus.SwerveLib.Utils;

import java.util.function.DoubleSupplier;

public class Smoothjoystick {

    private double exponent;

    public Smoothjoystick(double kJ){
        this.exponent = kJ;
    }

    public DoubleSupplier filter(DoubleSupplier input){
        return ()-> Math.signum(input.getAsDouble()) * Math.pow(Math.abs(input.getAsDouble()), exponent);
    }


}
