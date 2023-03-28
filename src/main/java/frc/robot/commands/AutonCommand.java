package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.arcade.ArcadeDriveSubsystem;



public class AutonCommand extends SequentialCommandGroup {
    //private ArcadeDriveSubsystem arcadeSubsystem;
    private Timer timer;
    private final ArcadeDriveSubsystem subsystem; 
    //private final IntakeSubsystem intakeSubsystem;

    public AutonCommand(ArcadeDriveSubsystem subsystem, double startTime) {
        timer = new Timer();
        this.subsystem = subsystem;
        //this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.subsystem);
        addCommands(
                // reset encoders
                new InstantCommand(subsystem::resetEncoders),
                new InstantCommand(timer::reset),
                // autobots, roll out
                new RunCommand(() -> {
                    //boolean a = false;
                    timer.start();
                    // while (timer.get() < 2) {
                    //     subsystem.halfPower();
                    // }
                double timeElapsed; 
                
                timeElapsed = timer.get()-startTime;
                SmartDashboard.putNumber("Timer: ", timer.get());
                SmartDashboard.putNumber("TimeElapsed: ", timeElapsed);
                SmartDashboard.putNumber("StartTime: ", startTime);
                if (timeElapsed >= 7) {
                    subsystem.drive(0);
                }
                else if (timeElapsed > 4) {
                    subsystem.drive(-0.5);
                }
                else if (timeElapsed > 1) {
                    subsystem.drive(0.3);
                }
                else if (timeElapsed >= 0){
                    subsystem.drive(-0.5);   
                }
                //    do {
                        //a = subsystem.moveToPosition(-35,0.4);
                  //  }
                    //while (!a);
                })
        );
    }

}
