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

    // now the buttons, who's description is purely in the eye of the beholder 
    
    public HOTASJoystick(int port) {
        super(port);
    }

}
