package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

public class IntakeChassis {
    private CANSparkMax intakeElevator;
    private CANSparkMax arm;
    public IntakeChassis(CANSparkMax intakeElevator, CANSparkMax armInput) {
        this.intakeElevator = intakeElevator;
        this.arm = armInput;
    }

    public CANSparkMax getIntakeElevator() { return intakeElevator; }

    public CANSparkMax getArm() { return arm; }
}
