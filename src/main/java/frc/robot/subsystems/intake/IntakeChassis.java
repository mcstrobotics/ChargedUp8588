package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

public class IntakeChassis {
    private CANSparkMax arm;
    private CANSparkMax intake;

    public IntakeChassis(CANSparkMax arm, CANSparkMax intake) {
        this.arm = arm;
        this.intake = intake;
    }

    public CANSparkMax getArm() { return arm; }

    public CANSparkMax getIntake() { return intake; }
}