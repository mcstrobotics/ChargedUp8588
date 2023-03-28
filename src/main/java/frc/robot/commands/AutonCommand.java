package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;



public class AutonCommand extends SequentialCommandGroup {
    private Timer timer;

    private final DriveSubsystem subsystem; 
    //private final IntakeSubsystem intakeSubsystem;

    public AutonCommand(DriveSubsystem subsystem, double startTime) {
        timer = new Timer();
        this.subsystem = subsystem;
        addRequirements(this.subsystem);
        addCommands(
                // reset encoders
                new InstantCommand(subsystem::resetEncoders),
                new InstantCommand(timer::reset),
                new InstantCommand(timer::start),
                // autobots, roll out
                new RunCommand(() -> {
                double timeElapsed = timer.get()-startTime;
                  
                SmartDashboard.putNumber("Timer: ", timer.get());
                if (timeElapsed >= 7) {
                    subsystem.drive(0);
                }
                else if (timeElapsed > 3.5) {
                    subsystem.drive(-0.5);
                }
                else if (timeElapsed > 0.75) {
                    subsystem.drive(0.4);
                }
                else if (timeElapsed >= 0){
                    subsystem.drive(-0.7);   
                }
                })
        );
    }

}
