
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT,
        BACK,
        STOP
    }
    public final Supplier<Double> armUp;
    public final Supplier<Double> armDown;
    public final Supplier<Boolean> intakeIn;
    public final Supplier<Boolean> intakeOut;

    public IntakeInputs(Supplier<Double> armUp, Supplier<Double> armDown, Supplier<Boolean> intakeIn, Supplier<Boolean> intakeOut){
        this.armUp = armUp;
        this.armDown = armDown;
        this.intakeIn = intakeIn;
        this.intakeOut = intakeOut;
    }
}
