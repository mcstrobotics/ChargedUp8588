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

package frc.robot.usercontrol;

import edu.wpi.first.wpilibj.Joystick;

public class HOTASJoystick extends Joystick{
    
    // defines the axes for each button, slider, and joystick on the flight pad
    public static final int STICK_AXIS_X = 0; // axis 0
    public static final int STICK_AXIS_Y = 0; //axis 2
    public static final int SMALL_BUTTON_X = 0; //axis 4
    public static final int SMALL_BUTTON_Y = 0; //axis 2
    public static final int TURNER_AXIS = 0; //axis 3

    // POV Button, whatever that is
    public static final int POV_BUTTON = 0; //POV ranges from 0 to 315 degrees

    // now the buttons, who's description is purely in the eye of the beholder 
    public static final int B_BUTTON = 0; //button 2, this is the button with a B on it on the top right side of the flight stick
    public static final int TRIGGER_BUTTON = 0; // button 0, this is the trigger button on the flight stick
    
    public HOTASJoystick(int port) {
        super(port);
    }

}
