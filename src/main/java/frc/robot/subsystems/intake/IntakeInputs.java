
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT,
        BACK,
        STOP
    }
    public final Supplier<Double> elevator;

    public final Supplier<Double> arm;
    public IntakeInputs(Supplier<Double> elevator, Supplier<Double> arm){
        this.elevator = elevator;
        this.arm = arm;
    }
}
