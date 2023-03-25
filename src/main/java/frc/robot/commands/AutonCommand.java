package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveDirection;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.tank.TankDriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;



public class AutonCommand extends SequentialCommandGroup {
    private Timer timer;
    private final DriveSubsystem subsystem;
    //private final IntakeSubsystem intakeSubsystem;

    public AutonCommand(DriveSubsystem subsystem) {
        this.subsystem = subsystem;
        //this.intakeSubsystem = intakeSubsystem;
        addRequirements(subsystem);
        addCommands(
                // reset encoders
                new InstantCommand(subsystem::resetEncoders),
                // autobots, roll out
                new RunCommand(() -> {
                    boolean a = false;
                    timer = new Timer();
                    timer.start();
                    while (timer.get() < 2) {
                        subsystem.halfPower();
                    }

                    subsystem.drive(0, DriveDirection.FORWARD);
                //    do {
                        //a = subsystem.moveToPosition(-35,0.4);
                  //  }
                    //while (!a);

                })
        );
    }

}
