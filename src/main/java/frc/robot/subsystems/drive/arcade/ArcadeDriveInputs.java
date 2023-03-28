// Abigail Prowse - adapted for a HOTAS input

package frc.robot.subsystems.drive.arcade;

import java.util.function.Supplier;

public class ArcadeDriveInputs {
    public final Supplier<Double> xStick;
    public final Supplier<Double> yStick;
    public final Supplier<Integer> pov;


    public ArcadeDriveInputs(Supplier<Double> xDirection, Supplier<Double> yDirection, Supplier<Integer> pov) {
        this.xStick = xDirection;
        this.yStick = yDirection;
        this.pov = pov;
    }
}
