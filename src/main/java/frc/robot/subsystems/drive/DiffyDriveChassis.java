/*
Name: Abigail Prowse
Program: ArcadeDriveChassis
This is the starting of the ArcadeDriveChassis.  Does not make complete sense yet and definitely needs work
Check out ArcadeDriveSubsystem and other subsystems.
Date: 3/29/2021
 */

package frc.robot.subsystems.drive;


import com.revrobotics.CANSparkMax;

public class DiffyDriveChassis {

    private CANSparkMax frontRight, frontLeft, backRight, backLeft;

    public DiffyDriveChassis(CANSparkMax frontRight, CANSparkMax frontLeft, CANSparkMax backRight, CANSparkMax backLeft) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    public CANSparkMax getFrontRight() {
        return frontRight;
    }

    public CANSparkMax getFrontLeft() {
        return frontLeft;
    }

    public CANSparkMax getBackRight() { return backRight; }

    public CANSparkMax getBackLeft() { return backLeft; }
}
