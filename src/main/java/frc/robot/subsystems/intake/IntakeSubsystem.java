package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {

    private final double intakePower = 0.50;

    private IntakeChassis chassis;
    private IntakeInputs inputs;

    public IntakeSubsystem(IntakeChassis chassis, IntakeInputs inputs){
        this.chassis = chassis;
        this.inputs = inputs;
    }

    @Override
    public void periodic() {
    }

    public void setPowers() {
        switch (this.inputs.arm.get()) {
            case FRONT:
                this.chassis.getArm().set(0.3);
            case BACK:
                this.chassis.getArm().set(-0.3);
            default:
                this.chassis.getArm().set(0);
        }

        switch (this.inputs.elevator.get()) {
            case FRONT:
                this.chassis.getIntakeElevator().set(0.3);
            case BACK:
                this.chassis.getIntakeElevator().set(-0.3);
            default:
                this.chassis.getIntakeElevator().set(0.2);
        }
    }
}
