/*
 * Made by: Abigail Prowse
 * Started on: March 7, 2023
 * Purpose: Class for gathering input data from HOTAS Flightstick
 * Package: frc.robot.usercontrol
 * 
 * Last Edited On: March 7, 2023
 * Change Log:
 * 3/7/23 - add STICK_AXIS_X and STICK_Y_AXIS
 */

 // Last Edited On: March 8, 2023 by Rohan Parikh
 // Change Log:
 // Extended Joystick class to allow for easier access to HOTAS flightstick
 // Joystick class already also comes with built in methods for getting axis and button values for flightstick

 // Last Edited On: March 16, 2023 by Abigail Prowse
 // Change Log:
 // Added every button port.
 // added getters for every

package frc.robot.usercontrol;

import edu.wpi.first.wpilibj.Joystick;

public class HOTASJoystick extends Joystick{
    
    // defines the axes for each button, slider, and joystick on the flight pad
    public static final int STICK_AXIS_X = 0; // axis 0
    public static final int STICK_AXIS_Y = 1; //axis 1
    public static final int SMALL_JOYSTICK_X = 2; //axis 2
    public static final int SMALL_JOYSTICK_Y = 3; //axis 3
    public static final int STICK_AXIS_Z_ROTATE = 4; // axis 4

    // POV Button, whatever that is
    public static final int POV_BUTTON = 0; //POV ranges from 0 to 315 degrees

    // now the buttons, who's description is purely in the eye of the beholder 
    public static final int TRIGGER_BUTTON = 0; // button 0, this is the trigger button on the flight stick
    public static final int A_BUTTON = 1; // button 1, this is the button with an a on it
    public static final int B_BUTTON = 2; //button 2, this is the button with a B on it on the top right side of the flight stick
    public static final int SMALL_JOYSTICK_BUTTON = 3; // button 3, this is the button under the small thumb joystick
    public static final int CIRCLE_BUTTON_BACK = 4; // button 4, this is the button on the back of the joystick that is a perfect gray circle
    public static final int PINKY_TRIGGER = 5; // button 5, this is the pinky trigger
    public static final int CIRCLE_D_PAD_UP = 6; // button 6, this is the circular D-PAD to the right of A button that has a concentric circle design. UP Direction
    public static final int CIRCLE_D_PAD_RIGHT = 7; // button 7, this is the circular D-PAD to the right of A button that has a concentric circle design. RIGHT Direction
    public static final int CIRCLE_D_PAD_DOWN = 8; // button 8, this is the circular D-PAD to the right of A button that has a concentric circle design. DOWN Direction
    public static final int CIRCLE_D_PAD_LEFT = 9; // button 9, this is the circular D-PAD to the right of A button that has a concentric circle design. LEFT Direction
    public static final int PLUS_D_PAD_UP = 10; // button 10, this is the plus shaped D-PAD.  UP Direction
    public static final int PLUS_D_PAD_RIGHT = 11; // button 11, this is the plus shaped D-PAD. RIGHT direction
    public static final int PLUS_D_PAD_DOWN = 12; // button 12, this is the plus shaped D-PAD. DOWN direction
    public static final int PLUS_D_PAD_LEFT = 13; // button 13, this is the plus shaped D-PAD LEFT direction
    
    public HOTASJoystick(int port) {
        super(port);
    }

    // get values for all of the joysticks and buttons

    public double getStickXAxis() {
        return getRawAxis(STICK_AXIS_X);
    }

    public double getStickYAxis() {
        // this is inverted 
        return -getRawAxis(STICK_AXIS_Y);
    }

    public double getSmallJoystickX() {
        return getRawAxis(SMALL_JOYSTICK_X);
    }

    public double getSmallJoystickY() {
        return getRawAxis(SMALL_JOYSTICK_Y);
    }

    public double getAxisZRotate() {
        return getRawAxis(STICK_AXIS_Z_ROTATE);
    }

    public boolean getTriggerButton() {
        return getRawButton(TRIGGER_BUTTON);
    }

    public boolean getAButton() {
        return getRawButton(A_BUTTON);
    }

    public boolean getBButton() {
        return getRawButton(B_BUTTON);
    }

    public boolean getSmallJoystickButton() {
        return getRawButton(SMALL_JOYSTICK_BUTTON);
    }

    public boolean getCircleButtonBack() {
        return getRawButton(CIRCLE_BUTTON_BACK);
    }

    public boolean getPinkyTrigger() {
        return getRawButton(PINKY_TRIGGER);
    }

    public boolean getCircleDPadUp() {
        return getRawButton(CIRCLE_D_PAD_UP);
    }

    public boolean getCircleDPadRight() {
        return getRawButton(CIRCLE_D_PAD_RIGHT);
    }

    public boolean getCircleDPadDown() {
        return getRawButton(CIRCLE_D_PAD_DOWN);
    }

    public boolean getCircleDPadLeft() {
        return getRawButton(CIRCLE_D_PAD_LEFT);
    }

    public boolean getPlusDPadUp() {
        return getRawButton(PLUS_D_PAD_UP);
    }

    public boolean getPlusDPadRight() {
        return getRawButton(PLUS_D_PAD_RIGHT);
    }

    public boolean getPlusDPadDown() {
        return getRawButton(PLUS_D_PAD_DOWN);
    }

    public boolean getPlusDPadLeft() {
        return getRawButton(PLUS_D_PAD_LEFT);
    }
}
