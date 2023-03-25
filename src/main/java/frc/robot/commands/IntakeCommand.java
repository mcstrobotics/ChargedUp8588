/****
 * Made by Tejas Mehta
 * Made on Monday, March 29, 2021
 * File Name: DriveCommand
 * Package: frc.team8588.commands*/
package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveDirection;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem subsystem;
    public IntakeCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setPowers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}