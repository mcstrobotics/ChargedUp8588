package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

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

        this.chassis.getArm().setIdleMode(CANSparkMax.IdleMode.kBrake);
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
        chassis.getIntake().set(-0.35);

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
        chassis.getIntake().setIdleMode(IdleMode.kBrake);

        // arm / elevator
        double armUp = inputs.armUp.get() * armMax;
        double armDown = inputs.armDown.get() * armMax;
        boolean intakeIn = inputs.intakeIn.get();
        boolean intakeOut = inputs.intakeOut.get();

        if (chassis.getIntake().getMotorTemperature() <= 65) {
            if (intakeIn) {
                // intake out
                //chassis.getIntake().set(intakeOutMax);
                intakeIn();
            } else if (intakeOut) {
                // intake out
                //chassis.getIntake().set(intakeInMax);
                intakeOut();
            } else {
                chassis.getIntake().set(0);
            }
            SmartDashboard.putString("Intake failure?", "NO");
            SmartDashboard.putBoolean("Intake In?", intakeIn);
            SmartDashboard.putBoolean("Intake Out?", intakeOut);
        } else {
            SmartDashboard.putString("Intake failure?", "YES");
        }

        if (inputs.armUp.get() > 0.2) {
            chassis.getArm().set(-armUp);
        }
        else if (inputs.armDown.get() > 0.2) {
            chassis.getArm().set(armDown);
        } else {
            chassis.getArm().set(0);
        }
    }
}