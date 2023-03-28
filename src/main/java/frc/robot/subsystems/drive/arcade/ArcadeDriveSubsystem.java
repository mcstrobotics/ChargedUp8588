/*
Name: Abigail Prowse
Program: ArcadeDriveSubsytem
This is the starting of the ArcadeDriveSubsytem.  Does not make complete sense yet and definitely needs work
Check out ArcadeDriveChassis and other subsystems.
Date: 3/29/2021
 */

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.arcade;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveDirection;
import frc.robot.subsystems.drive.DriveSubsystem;


public class ArcadeDriveSubsystem implements DriveSubsystem {
    private ArcadeDriveChassis chassis;
    private ArcadeDriveInputs inputs;
    private PowerDistribution pdh;
    private final MotorControllerGroup leftDriveGroup;
    private final MotorControllerGroup rightDriveGroup;
    private final DifferentialDrive drive;
    private double power = 1;

    public ArcadeDriveSubsystem(ArcadeDriveChassis chassis, ArcadeDriveInputs inputs) {
        this.chassis = chassis;
        this.inputs = inputs;
        this.pdh = new PowerDistribution();
        chassis.getBackRight().setInverted(true);
        chassis.getFrontRight().setInverted(true);
        this.leftDriveGroup = new MotorControllerGroup(chassis.getBackLeft(), chassis.getFrontLeft());
        this.rightDriveGroup = new MotorControllerGroup(chassis.getBackRight(), chassis.getFrontRight());
        this.drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

        this.setBrake();
    }
    public void drive(double x){
        drive.arcadeDrive(x,0);

        SmartDashboard.putNumber("Front Right Temp: ", chassis.getFrontRight().getMotorTemperature());
        SmartDashboard.putNumber("Front Left Temp: ", chassis.getFrontLeft().getMotorTemperature());
        SmartDashboard.putNumber("Back Right Temp: ", chassis.getBackRight().getMotorTemperature());
        SmartDashboard.putNumber("Back Left Temp: ", chassis.getBackLeft().getMotorTemperature());

        SmartDashboard.putNumber("Total Current Draw: ", pdh.getTotalCurrent());
        SmartDashboard.putNumber("Total Power Draw: ", pdh.getTotalPower());
        

    }
    public void drive(){ 
        drive.arcadeDrive(inputs.yStick.get(), inputs.xStick.get());
    }

    public void manual_drive(double power, DriveDirection direction) {
        // switch (direction)
        // {
        //     case FORWARD:
        //         setMotors(power, power);
        //         break;

        //     case BACKWARD:
        //         setMotors(-power, -power);
        //         break;

        //     case RIGHT:
        //         setMotors(power, -power);
        //         break;

        //     case LEFT:
        //         setMotors(-power, power);
        //         break;
        // }
    }

    @Override
    public void drive(double leftX, double leftY, double rightX, double rightY) {
        /*
        double direction;
        // calculates the direction off of left joystick values
        if ((Math.abs(leftX) < 0.1 && Math.abs(leftY) < 0.1) {
            // center
            direction = 0;
            // sets direction to 0, meaning there is no movement.
        }
        else if (leftX > 0 && leftY > 0) {
            // first quadrant
            direction = Math.atan(leftY/leftX);
            // uses the arctangent of the triangle made by the x and y values to find the angle to drive in.
        }
        else if (leftX < 0 && leftY > 0) {
            direction = Math.atan(leftY/leftX) + Math.PI * 3 / 2;
            // uses the arctangent of the triangle made by the x and y values to find the angle to drive in. + 270 degrees.
        }
        else if (leftX < 0 && leftY < 0) {
            // third quadrant
            direction = Math.atan(leftY/leftX) + Math.PI;
            // uses the arctangent of the triangle made by the x and y values to find the angle to drive in. + 180 degrees.
        }
        else if (leftX > 0 && leftY < 0) {
            direction = Math.atan(leftY/leftX) + Math.PI / 2;
            // uses the arctangent of the triangle made by the x and y values to find the angle to drive in. + 90 degrees
        }

        double speed = Math.sqrt(leftX * leftX + leftY * leftY);
        // calculates the speed by the hypotenuse of the said triangle.
        */

//        chassis.getLeft.setLeft(leftX + leftY);
//        chassis.getRight.setRight(leftX - leftY);
    }

    @Override
    public void resetEncoders() {
        chassis.getFrontLeft().getEncoder().setPosition(0);
        chassis.getFrontRight().getEncoder().setPosition(0);

        chassis.getBackLeft().getEncoder().setPosition(0);
        chassis.getBackRight().getEncoder().setPosition(0);
    }

    @Override
    public boolean moveToPosition(double location, double speed) {
        if(location > 0)
            manual_drive(speed, DriveDirection.FORWARD);
        else
            manual_drive(speed, DriveDirection.BACKWARD);

        return Math.abs(chassis.getBackLeft().getEncoder().getPosition()) > Math.abs((int)location); //use the back left encoder for position tracking (driving in a straight line, doesn't really matter which one we use for arcade)
    }

    @Override
    public boolean moveToPosition(PIDController pid, double location, double speed) {
        double pidSpeed = pid.calculate(chassis.getBackLeft().getEncoder().getPosition(), location) * speed;
        manual_drive(pidSpeed, DriveDirection.BACKWARD);

        return Math.abs(chassis.getBackLeft().getEncoder().getPosition()) > Math.abs((int)location);
    }

    @Override
    public boolean strafeToPosition(double location, double speed) {
        return false;
    }

    @Override
    public boolean strafeToPosition(PIDController pid, double location, double speed) {
        return false;
    }

    @Override
    public void setPowers() {
        double forward = inputs.yStick.get();
        double turn = inputs.xStick.get();
        double pov = inputs.pov.get();

        double triggerThreshold = 0.3;


        //forward = forward * forward * forward;
        //turn = turn * turn * turn;
        // curve movement

        chassis.getBackLeft().set(-(forward - turn));
        chassis.getFrontLeft().set(-(forward - turn));

        chassis.getBackRight().set((forward + turn));
        chassis.getFrontRight().set(-(forward + turn));

        
    }

    public void setPowersFO(AHRS ahrs) {
        // do nothing
    }

    public double returnCurrentDraw() {
        return 0;
    }

    // method needs to take in x and y of one joystick.  Also needs to take in power.
    // then needs to change direction according to that.

    // Return the power draw of all four motors.
    @Override
    public void periodic(){
        setPowers();

        SmartDashboard.putNumber("Front Right Temp: ", chassis.getFrontRight().getMotorTemperature());
        SmartDashboard.putNumber("Front Left Temp: ", chassis.getFrontLeft().getMotorTemperature());
        SmartDashboard.putNumber("Back Right Temp: ", chassis.getBackRight().getMotorTemperature());
        SmartDashboard.putNumber("Back Left Temp: ", chassis.getBackLeft().getMotorTemperature());

        SmartDashboard.putNumber("Total Current Draw: ", pdh.getTotalCurrent());
        SmartDashboard.putNumber("Total Power Draw: ", pdh.getTotalPower());

        SmartDashboard.putBoolean("Front Right: ", chassis.getFrontRight().getInverted());
        SmartDashboard.putBoolean("Front Left: ", chassis.getFrontLeft().getInverted());
        SmartDashboard.putBoolean("Back Right: ", chassis.getBackRight().getInverted());
        SmartDashboard.putBoolean("Back Left: ", chassis.getBackLeft().getInverted());
    }


    @Override
    public double debug() {
        return 0;
    }

    public void setBrake() {
        chassis.getBackLeft().setIdleMode(CANSparkMax.IdleMode.kBrake);
        chassis.getBackRight().setIdleMode(CANSparkMax.IdleMode.kBrake);
        chassis.getFrontLeft().setIdleMode(CANSparkMax.IdleMode.kBrake);
        chassis.getFrontRight().setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setCoast() {
        chassis.getBackLeft().setIdleMode(CANSparkMax.IdleMode.kCoast);
        chassis.getBackRight().setIdleMode(CANSparkMax.IdleMode.kCoast);
        chassis.getFrontLeft().setIdleMode(CANSparkMax.IdleMode.kCoast);
        chassis.getFrontRight().setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void halfPower() {
        power = 0.5;
    }

    public void fullPower() {
        power = 1;
    }

}

