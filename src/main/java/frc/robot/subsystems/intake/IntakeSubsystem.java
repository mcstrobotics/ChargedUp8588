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
<<<<<<< HEAD
        //SmartDashboard.putNumber("Shooter RPM over time", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        //SmartDashboard.putNumber("Current Shooter RPM for Dial", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        //SmartDashboard.putNumber("Shooter Temperature", chassis.getShooter().getMotorTemperature());
<<<<<<< HEAD
<<<<<<< HEAD
=======
        SmartDashboard.putNumber("Shooter RPM over time", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        SmartDashboard.putNumber("Current Shooter RPM for Dial", Math.abs(chassis.getShooter().getEncoder().getVelocity()));
        SmartDashboard.putNumber("Shooter Temperature", chassis.getShooter().getMotorTemperature());
    }
>>>>>>> parent of 28a918d... intake stuff

    public void runFlywheel(boolean on) {
        if(on) {
            chassis.getShooter().set(-1);
        }
        else {
            chassis.getShooter().set(0);
        }
=======

>>>>>>> parent of 8d0cb48... Revert "Merge branch 'master' of https://github.com/mcstrobotics/ChargedUp8588 into master"
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

<<<<<<< HEAD
    public void armDown() {
        chassis.getArm().set(-armPower);
<<<<<<< HEAD
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
=======
    public void intakeIn() {
        chassis.getLeft().set(intakePower);
        chassis.getRight().set(-intakePower);
        chassis.getIndexer().set(-0.35);
    }

    public void intakeOut() {
        chassis.getLeft().set(-intakePower);
        chassis.getRight().set(intakePower);
        chassis.getIndexer().set(0.35);

>>>>>>> parent of 28a918d... intake stuff
=======
>>>>>>> parent of 8d0cb48... Revert "Merge branch 'master' of https://github.com/mcstrobotics/ChargedUp8588 into master"
    }

    public void armUp() {
        chassis.getArm().set(armPower);
    }

    public void armDown() {
        chassis.getArm().set(-armPower);
    }
}
