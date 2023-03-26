/**
 * Made by Tejas Mehta
 * Made on Monday, March 29, 2021
 * File Name: TankDriveSubsystem
 * Package: frc.team8588.subsystems.drive*/
package frc.robot.subsystems.drive.tank;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.subsystems.drive.DriveDirection;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TankDriveSubsystem implements DriveSubsystem {

    private TankDriveChassis chassis;
    private TankDriveInputs inputs;
    //private DifferentialDrive drive;

    private static final double functionEndPoint = 0.5;
    private double accelAmount = 0.001;
    private double deccelAmount = 0.05; 
    private double targetAmount = 0;
    private double currentAmount = 0;
    private double power = 1;

    public TankDriveSubsystem(TankDriveChassis chassis, TankDriveInputs inputs) {
        this.chassis = chassis;
        this.inputs = inputs;
        //creating 2 direction differential drive with motor controller groups
        /*
        this.drive = new DifferentialDrive(
                new MotorControllerGroup(
                        chassis.getFrontLeft(),
                        chassis.getBackLeft()
                ),
                new MotorControllerGroup(
                        chassis.getFrontRight(),
                        chassis.getBackRight()
                )
        );
        */

    }

    @Override
    public void manual_drive(double power, DriveDirection direction) {
        /*
        switch (direction) {
            case FORWARD:
                chassis.getLeft().set(power);
                chassis.getRight().set(-power);
                break;
            case BACKWARD:
                chassis.getLeft().set(-power);
                chassis.getRight().set(power);
                break;
            case LEFT:
            case TURN_CCW:
                chassis.getLeft().set(-power);
                chassis.getRight().set(-power);
                break;
            case RIGHT:
            case TURN_CW:
                chassis.getLeft().set(power);
                chassis.getRight().set(power);
                break;
        }
        */
    }

    public double getCurInput(double x) {
        targetAmount = x;
        long curTime = System.currentTimeMillis();
        if (curTime % 10 == 0 && Math.abs(targetAmount - currentAmount) >= accelAmount) {
            if (targetAmount < currentAmount) {
                currentAmount -= deccelAmount;
                //JOSHUA, ALEXEI AND NEIL WERE HERE
                //LUKE WAS HERE (Again)
                //MATTHIAS WAS HERE
                // ABIGAIL WAS HERE (HI!)
                //JAKUB WAS HERE
            }else if (targetAmount > currentAmount) {
                currentAmount += accelAmount;
            }
        }
        return targetAmount;
        /*double finReturn = 0;

        long neededTime = curTime + (long)(accelAmount * 1000);
        while (System.currentTimeMillis() < neededTime) {
            if (Math.abs(x) < functionEndPoint) {
                finReturn = 4 * Math.pow(x * ((System.currentTimeMillis() - curTime) / accelAmount * 1000), 3); // Gets the percentage of time to desirted input.
            }else {
                finReturn = x;
            }
        }
        if (Math.abs(x) < functionEndPoint) {
            finReturn = 4 * Math.pow(x, 3);
        }else {
            finReturn = x;
        }
        return finReturn;*/
        /*execute.schedule(() -> {
            curInput -
        }, 10, TimeUnit.MILLISECONDS);*/
    }

    @Override
    public void setPowers() {
        /*chassis.getFrontLeft().set(getCurInput(-inputs.rightStickY.get()));
        chassis.getBackLeft().set(getCurInput(-inputs.rightStickY.get()));

        chassis.getFrontRight().set(getCurInput(inputs.leftStickY.get()));
        chassis.getBackRight().set(getCurInput(inputs.leftStickY.get()));
        System.out.println("Current power for left: " + inputs.leftStickY.get());*/

        //putting input on a curve to prevent jerky movements
        double left = inputs.leftStickY.get();
        double right = inputs.rightStickY.get();

        boolean leftNegative = left < 0;
        boolean rightNegative = right < 0;

        left = (left * left * left) / 4;
        //if(leftNegative)
            //left *= -1;

        right = (right * right * right) / 4;
        //if(rightNegative)
            //right *= -1;
        /*
        chassis.getBackLeft().set(inputs.leftStickY.get());
        chassis.getBackRight().set(-inputs.rightStickY.get());
        chassis.getFrontLeft().set(inputs.leftStickY.get());
        chassis.getFrontRight().set(-inputs.rightStickY.get());
        */

        chassis.getBackLeft().set(left);
        chassis.getFrontLeft().set(left);

        chassis.getBackRight().set(-right);
        chassis.getFrontRight().set(-right);
        //drive.tankDrive(left, right);
    }

    public void setPowersFO(AHRS ahrs) {
        // do nothing
    }

    @Override
    public void drive(double leftX, double leftY, double rightX, double rightY) {

    }

    @Override
    public void resetEncoders() {

    }

    @Override
    public boolean moveToPosition(double location, double speed) {
        return false;
    }

    @Override
    public boolean moveToPosition(PIDController pid, double location, double speed) {

        return false;
    }

    @Override
    public boolean strafeToPosition(double location, double speed) {
        return false;
    }

    @Override
    public boolean strafeToPosition(PIDController pid, double location, double speed) {
        return false;
    }

    public double returnCurrentDraw() {
        return chassis.getBackLeft().getOutputCurrent() + chassis.getBackRight().getOutputCurrent() + chassis.getFrontLeft().getOutputCurrent() + chassis.getFrontRight().getOutputCurrent();
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
//jacob
