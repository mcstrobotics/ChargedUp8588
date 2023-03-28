/****
 * Made by Tejas Mehta
 * Made on Monday, March 29, 2021
 * File Name: DriveSubsystem
 * Package: frc.team8588.subsystems*/
package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveSubsystem extends Subsystem {

    void resetEncoders(); //reset all encoders for the drive chassis

    void setPowers();

    void drive(double x);

    void setBrake();

    void setCoast();

    void sendTelemetry();
}
