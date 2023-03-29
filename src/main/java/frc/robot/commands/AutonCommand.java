package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemInterface;



public class AutonCommand extends SequentialCommandGroup {
    private Timer timer;

    private final DriveSubsystemInterface subsystem; 
    //private final IntakeSubsystem intakeSubsystem;

    public AutonCommand(DriveSubsystemInterface subsystem, double startTime) {
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
                else if (timeElapsed > 1.5) {
                    subsystem.drive(-0.5);
                }
                else if (timeElapsed > 0.3) {
                    subsystem.drive(0.4);
                }
                else if (timeElapsed >= 0){
                    subsystem.drive(-0.7);   
                }
                })
        );
    }

}
