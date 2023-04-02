// Abigail Prowse - adapted for a HOTAS input

package frc.robot.subsystems.drive;

import java.util.function.Supplier;

public class DiffyDriveInputs {
    public final Supplier<Double> xStick;
    public final Supplier<Double> yStick;

    public DiffyDriveInputs(Supplier<Double> xDirection, Supplier<Double> yDirection) {
        this.xStick = xDirection;
        this.yStick = yDirection;
    }
}
