
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT,
        BACK,
        STOP
    }
    public final Supplier<Double> arm;
    public final Supplier<Double> intake;

    public IntakeInputs(Supplier<Double> arm, Supplier<Double> intake){
        this.arm = arm;
        this.intake = intake;
    }
}
