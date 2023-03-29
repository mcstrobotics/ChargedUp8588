/****
 * Made by Tejas Mehta
 * Made on Monday, March 22, 2021
 * File Name: Gamepad
 * Package: frc.team8588.controller*/
package frc.robot.usercontrol;

import edu.wpi.first.wpilibj.Joystick;

public class GamepadF310 {

    /**
     * Gamepad mappings as the F310 controller isn't officially in the WPI repo.
     * Each Number corresponds to a specific "axis" or "button" in the joystick class.
     * The variable names should be pretty self-explanatory as to what each number
     * represents.
     */
    public static final int GAMEPAD_LEFT_X = 0;
    public static final int GAMEPAD_LEFT_Y = 1;
    public static final int GAMEPAD_RIGHT_X = 2;
    public static final int GAMEPAD_RIGHT_Y = 3;

    public static final int GAMEPAD_LEFT_TRIGGER = 4;
    public static final int GAMEPAD_RIGHT_TRIGGER = 5; // not sure if these are right

    public static final int GAMEPAD_A = 1;
    public static final int GAMEPAD_B = 2;
    public static final int GAMEPAD_X = 3;
    public static final int GAMEPAD_Y = 4;

    public static final int GAMEPAD_LEFT_BUMPER = 5;
    public static final int GAMEPAD_RIGHT_BUMPER = 6;


    public Joystick joystick;
    public GamepadF310(int port) {
        joystick = new Joystick(port);
    }

    public double getLeftX() {
        return joystick.getRawAxis(GAMEPAD_LEFT_X);
    }

    public double getLeftY() {
        return joystick.getRawAxis(GAMEPAD_LEFT_Y);
    }

    public double getRightX() {
        return joystick.getRawAxis(GAMEPAD_RIGHT_X);
    }

    public double getRightY() {
        return joystick.getRawAxis(GAMEPAD_RIGHT_Y);
    }

    public double getRightTrigger() {
        return joystick.getRawAxis(GAMEPAD_RIGHT_TRIGGER);
    }

    public boolean rightTriggerPressed() { return joystick.getRawAxis(GAMEPAD_RIGHT_TRIGGER) > 0.3; }

    public double getLeftTrigger() {
        return joystick.getRawAxis(GAMEPAD_LEFT_TRIGGER);
    }

    public boolean leftTriggerPressed() { return joystick.getRawAxis(GAMEPAD_LEFT_TRIGGER) > 0.3; }

    public boolean getA() {
        return joystick.getRawButton(GAMEPAD_A);
    }

    public boolean getB() {
        return joystick.getRawButton(GAMEPAD_B);
    }

    public boolean getX() {
        return joystick.getRawButton(GAMEPAD_X);
    }

    public boolean getY() {
        return joystick.getRawButton(GAMEPAD_Y);
    }

    public boolean getLeftBumper() {
        return joystick.getRawButton(GAMEPAD_LEFT_BUMPER);
    }

    public boolean getRightBumper() {
        return joystick.getRawButton(GAMEPAD_RIGHT_BUMPER);
    }

}
