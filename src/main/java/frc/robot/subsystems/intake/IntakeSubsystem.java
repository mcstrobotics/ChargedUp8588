package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {

    private final double intakePower = 0.50;
    private final double armPower = 0.5;

    private IntakeChassis chassis;

    public IntakeSubsystem(IntakeChassis chassis){
        this.chassis = chassis;
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Shooter RPM over time", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        //SmartDashboard.putNumber("Current Shooter RPM for Dial", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        //SmartDashboard.putNumber("Shooter Temperature", chassis.getShooter().getMotorTemperature());
<<<<<<< HEAD

    }

    // Stops all two motors in the arm
    public void stopAll() {
        chassis.getArm().set(0);
        chassis.getIntake().set(0);
    }

    public void intakeIn() {
        chassis.getIntake().set(intakePower);
    }

    public void intakeOut() {
        chassis.getIntake().set(0.35);

    }

    public void armUp() {
        chassis.getArm().set(armPower);
    }

    public void armDown() {
        chassis.getArm().set(-armPower);
=======

    }

    // Stops all two motors in the arm
    public void stopAll() {
        chassis.getArm().set(0);
        chassis.getIntake().set(0);
    }

    public void intakeIn() {
        chassis.getIntake().set(intakePower);
    }

    public void intakeOut() {
        chassis.getIntake().set(0.35);

>>>>>>> parent of 8c5e0bf... Merge branch 'master' of https://github.com/mcstrobotics/ChargedUp8588 into master
    }

    public void armUp() {
        chassis.getArm().set(armPower);
    }

    public void armDown() {
        chassis.getArm().set(-armPower);
    }
}
