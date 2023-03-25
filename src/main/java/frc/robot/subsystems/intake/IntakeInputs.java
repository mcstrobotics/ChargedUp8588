
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT,
        BACK,
        STOP
    }
    public final Supplier<Boolean> elevatorUp;
    public final Supplier<Boolean> elevatorDown;

    public final Supplier<Boolean> armUp;
    public final Supplier<Boolean> armDown;

    public IntakeInputs(Supplier<Boolean> elevatorUp, Supplier<Boolean> elevatorDown, Supplier<Boolean> armUp, Supplier<Boolean> armDown){
        this.elevatorUp = elevatorUp;
        this.elevatorDown = elevatorDown;
        this.armUp = armUp;
        this.armDown = armDown;
    }
}
