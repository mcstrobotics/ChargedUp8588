// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.tank.TankDriveChassis;
import frc.robot.subsystems.drive.tank.TankDriveSubsystem;
import frc.robot.subsystems.drive.tank.TankDriveInputs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.usercontrol.HOTASJoystick;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // HOTAS Flight Stick
  private HOTASJoystick flightStick = new HOTASJoystick(0); // change to correct port in driver station

  private DriveSubsystem driveSubsystem = new TankDriveSubsystem(
    new TankDriveChassis( // tank chassis as opposed to arcade or mecanum
      new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless), // first number corresponds with device id - may change
      new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless),
      new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless),
      new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
      ),
    new TankDriveInputs(flightStick::getX, flightStick::getY)); // x and y of 
    
  private DriveCommand driveCommand = new DriveCommand(driveSubsystem); // issue the drive commands from the drive subsystem
  // eventually make an auton command

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(AHRS ahrs) {
    // Configure the trigger bindings
    configureBindings(ahrs);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule commands tied to buttons

    // for each button number (corresponds to a button), we are going to run a command / method
    
    new JoystickButton(flightStick, 0); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public HOTASJoystick getGamepad() {
    return flightStick;
  }

  /**
  * Use this to pass the teleop command to the main {@link Robot} class.
  *
  * @return the command to run in teleop
  */
  public DriveCommand getDriveCommand()
  {
    // driveCommand will run in teleop
    return driveCommand;
  }
}
