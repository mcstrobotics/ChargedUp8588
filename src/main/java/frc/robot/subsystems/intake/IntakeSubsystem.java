package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {

    private final double intakePower = 0.50;
    private final double armPower = 0.5;

    private IntakeChassis chassis;
    private IntakeInputs inputs;

    public IntakeSubsystem(IntakeChassis chassis, IntakeInputs inputs){
        this.chassis = chassis;
        this.inputs = inputs;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM over time", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        SmartDashboard.putNumber("Current Shooter RPM for Dial", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        SmartDashboard.putNumber("Shooter Temperature", chassis.getShooter().getMotorTemperature());
    }

    public void runFlywheel(boolean on) {
        if(on) {
            chassis.getShooter().set(-1);
        }
        else {
            chassis.getShooter().set(0);
        }
    }

    public void runIndexer(boolean on) {
        if(on) {
            chassis.getIndexer().set(-1);
        }
        else
            chassis.getIndexer().set(0);
    }

    public void runFlywheelHIGH() {
        chassis.getShooter().set(-0.83);
    }

    public void runFlywheelLOW() {
        chassis.getShooter().set(-0.4);
    }

    public void runIndexer() {
        chassis.getIndexer().set(-1);
    }

    // Stops all four motors involved in Intake + Indexing + Shooting
    public void stopAll() {
        chassis.getRight().set(0);
        chassis.getLeft().set(0);
        chassis.getIndexer().set(0);
        chassis.getShooter().set(0);
    }

    public void intakeIn() {
        chassis.getLeft().set(intakePower);
        chassis.getRight().set(-intakePower);
        chassis.getIndexer().set(-0.35);
    }

    public void intakeOut() {
        chassis.getLeft().set(-intakePower);
        chassis.getRight().set(intakePower);
        chassis.getIndexer().set(0.35);

    }

    public void armUp() {
        chassis.getArm().set(armPower);
    }

    public void armDown() {
        chassis.getArm().set(-armPower);
    }
}
