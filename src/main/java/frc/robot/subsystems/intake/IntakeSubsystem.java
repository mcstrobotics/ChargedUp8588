package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {

    private final double intakePower = 1;
    private final double armPower = 0.5;

    private IntakeChassis chassis;
    private IntakeInputs inputs;

    public IntakeSubsystem(IntakeChassis chassis, IntakeInputs inputs){
        this.chassis = chassis;
        this.inputs = inputs;
        this.chassis.getIntake().setOpenLoopRampRate(0);
        this.chassis.getIntake().setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.chassis.getIntake().setOpenLoopRampRate(0);
        this.chassis.getArm().setOpenLoopRampRate(0.2);
        this.chassis.getIntake().setSmartCurrentLimit(12, 12);
    }

    @Override
    public void periodic() {}

    // Stops all two motors in the arm
    public void stopAll() {
        chassis.getArm().set(0);
        chassis.getIntake().set(0);
    }

    public void stopElevator() {
        chassis.getArm().set(0);
    }

    public void stopArm() {
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
    }

    public void setPowers() {
        // utilize each function based on inputs

        SmartDashboard.putNumber("Intake Temp", chassis.getIntake().getMotorTemperature());

        double armMax = 0.2;
        double intakeInMax = 0.35;
        double intakeOutMax = 0.5; 

        // arm / elevator
        double arm = inputs.arm.get() * armMax;
        double intake = inputs.intake.get();

        if (chassis.getIntake().getMotorTemperature() <= 65) {
            if (intake > 0.05) {
                // intake out
                chassis.getIntake().set(intake * intakeOutMax);
            } else if (intake < -0.05) {
                // intake out
                chassis.getIntake().set(intake * intakeInMax);
            } else {
                chassis.getIntake().set(0);
            }
            SmartDashboard.putString("Intake failure?", "NO");
        } else {
            SmartDashboard.putString("Intake failure?", "YES");
        }

        chassis.getArm().set(arm);
    }
}