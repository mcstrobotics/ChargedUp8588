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
import frc.robot.subsystems.drive.arcade.ArcadeDriveSubsystem;



public class AutonCommand extends SequentialCommandGroup {
    private ArcadeDriveSubsystem arcadeSubsystem;
    private Timer timer;
    private final DriveSubsystem subsystem;
    //private final IntakeSubsystem intakeSubsystem;

    public AutonCommand(DriveSubsystem subsystem, double startTime) {
        this.subsystem = subsystem;
        //this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.subsystem);
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
                double timeElapsed = timer.get() - startTime;
                if (timeElapsed < 3) {
                    subsystem.drive(0.6, DriveDirection.FORWARD);
                }
                else {
                    subsystem.drive(0, DriveDirection.FORWARD);
                }
                //    do {
                        //a = subsystem.moveToPosition(-35,0.4);
                  //  }
                    //while (!a);

                })
        );
    }

}
