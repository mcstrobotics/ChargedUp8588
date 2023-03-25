/*
* THIS CLASS IS NOT BEING USED
*/

package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT,
        BACK,
        STOP
    }
    public final Supplier<MoveStatus> elevator;

    public final Supplier<MoveStatus> arm;
    public IntakeInputs(Supplier<MoveStatus> elevator, Supplier<MoveStatus> arm){
        this.elevator = elevator;
        this.arm = arm;
    }
}
