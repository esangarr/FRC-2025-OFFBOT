package lib.ForgePlus.SwerveLib.Utils;

import java.util.function.DoubleSupplier;

/**
   * Constructs a SwerveModuleStateSupplier.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module
*/
public record SwerveModuleStateSupplier(DoubleSupplier speedMetersPerSecond, DoubleSupplier angle) {}