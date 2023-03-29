/****
 * Made by Tejas Mehta
 * Made on Monday, March 29, 2021
 * File Name: DriveCommand
 * Package: frc.team8588.commands*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystemInterface;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DriveCommand extends CommandBase {

    private DriveSubsystemInterface subsystem;
    private IntakeSubsystem intake;
    double power = 0.0;
    double lPower = 0.0;
    double rPower = 0.0;
    public DriveCommand(DriveSubsystemInterface subsystem, IntakeSubsystem intake) {
        this.subsystem = subsystem;
        this.intake = intake;
        addRequirements(subsystem, intake);
    }

    @Override
    public void execute() {
        intake.setPowers();
        
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setLeft(double power) {
        lPower = power;
    }

    public void setRight(double power){
        rPower = power;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
