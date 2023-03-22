3/1/2023

AUTONOMOUS PSEUDOCODE + PLANNING

STILL UNDER DEVELOPMENT 

Please don't accidentally or purposely nuke the document ;-; 

KEY ✨🔑

-   Priority step

-   Important Mechanics 

-   General Needed Stuff

-   Optional(if we have time)

-   Text in Red Means Step is Complete

* * * * *

OVERVIEW 👆

-   Steps 👞

-   Drop the pre-load into the station

-   Get out of the "safe zone" 

-   Locate the Doc and Generate a Path

-   Move Toward The Dock

-   Get on the Dock

-   Level 

Objective: Drop the payload and get out of the safe zone and make way to the dock 

Priority: Drop and get out 💨

* * * * *

PSEUDOCODE 

-   Break-Down  💡

-   Configure Camera 📹

-   Start Internal Timer ⏲️

-   Initialize Phase(step we are on to one)

-   While Loop (while the time elapsed on the timer is less than 15 seconds[autonomous period]

-   Switch Statement(if we are constantly checking timer) or just go through each phase one by one, it depends on how the program flow works tbh :/ 

-   Pre-Phase Stuff

-   Phase One: Drop the Preload Station in the doc

-   Locate the Dock Pad

-   Move Toward the Dock Pad

-   Drop the Preload into the Thing 

-   Change Phase to Two

-   Phase 2: Get Out of the Safe Zone 

-   Re-orientate

-   Move Out

-   Periodically check for obstacles(if one is identified, just go in a circle-square around it and continue on the path 

-   Change to Phase 3

-   Phase 3: Locate the Dock and Generate a Path

-   Locate the Dock on the Camera (sensor training) 

-   In the event it isn't visible just move around ig until it is seen

-   Identify corners and sides maybe and find point to move to (we want to be on the long side of the dock

-   Change to Phase 4 

-   Phase 4: Move Toward The Dock

-   Generate Path

-   Straight lines, L shape, etc)

-   Move periodically checking for objects and moving around them

-   Once either distance is traveled or the doc is in vicinity(depends on what approach we do) change to phase 5

-   Phase 5: Get on the Dock

-   Strafe around the horizontal edge, judging contours of objects

-   If there are no robots detected on the dock or we have a wide enough opening 

-   Switch gear(enhancement) to slow and move onto the dock (forward)

-   Switch to Phase 6

-   Phase 6: Level 

-   Somewhat smart figure this out idk 

-   Switch to Manual once time is up or autonomous mode is overridden

* * * * *

-   Mechanics and Functions Needed ❗

-   Auton People

-   Camera Configuration 📸

-   Configure the camera to be able to use locations on the camera to judge positioning of objects 

-   For the camera coding we hope to use OpenCV 

-   We are most likely going to use the camera class 

-   TO DO:

-   Once we obtain camera calculate FOV

-   Once target recognition is enabled, we can make calculations for translating the target position, pitch, yaw, etc. 

-   [Distance Judging](https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/identifying-and-processing-the-targets.html)  📏

-   How far is an object from the robot

-   Ideally be able to judge the distance between objects as well 

-   Identify Shape 🔺

-   Can probably have a parameter to specify what specifically we are looking out for and go through a directory to see which specific traits to watch out for 

-   Move Around Object/Identify Obstacle 🚨 

-   Includes a function of being able to sandwich between two robots 

-   Mix of distance judging, identify shape, identify obstacle 

-   For robots we can probably use aspect ratio and color of their bumpers (not same aspect ratio of line or payload station if bumper size isn't consistent) as well as the color of the different payloads 

-   If the robot is to run into something while it reroutes, actually i don't want to think about that >_> 

-   To find out if something is directly in the robot's path we can make a range of x and y values on the camera to represent what is directly one (robot chassis? Intake grab?) to see if it is directly in the robot's path! 

-   Generate Path ✍️

-   Generates most efficient way to get from point A to point B, still deciding whether we want to go in a straight line or an L shape like pattern 

-   Move on Path ⚡

-   Be able to move on the path, uses stuff from move around object/identify obstacle and is able to get back on their path (get angle to move and probable distance idk) 

-   Move x amount of meters

-   Scan for obstacles

-   If we see a contour that is not a long line on the ground

-   Move around it (for simplicity how about a x meter square) [we will have displaced x meters] 

-   Otherwise keep moving on the path  

-   Continue moving until either

-   A) we moved the proper distance (record distance traveled on path

-   B) We can tell that we are directly in front of the object 

-   Functions/Mechanics Needed from Tele-Op People (idk what we have and don't have)

-   Arm dropping (could also be autonomous's responsibility idk)

-   Release the payload, for autonomous we aren't planning on picking up objects

-   Strafing 

-   In order to find a place to get on the dock 

-   Rotate (be able to rotate the robot by a certain amount)

-   Should also have a getter function to know the orientation of the robot

-   Change Gear (enhancement)

-   Be able to switch how fast the robot will go, mainly used for finer motions to get on the dock/level

* * * * *

-   To Do List ✅

3/20 - 3/24

-   Monday: 

-   Pathing Algorithm 

-   Fill in the blanks for the autonomous parts 

-   Set alliance function

-   Locate dock pad 

-   Locate line 

-   Motor stuff

-   Tuesday:

-   Finish filling in blanks for autonomous parts 

-   Make sure intake payload doesn't interfere with scanning for obstacles 

-   Add a second if statement in the scanStationaryObjects method to make sure the object isn't within the bounds of the intake and that the payload is equipped 

-   Drive testing probably

-   Wednesday

-   Test bugs 

-   Finish necessary parts 

-   Thursday?

-   CRUNCH 

-   Friday

-   Do necessary tests at comp idk 

* * * * *

Misc 📝

-   Points to Research (Work In Progress) 🔎

-   Camera/Vision Processing

-   [Tutorial, C++ can probably be transferred to Java, we mainly need to know fundamentals](https://www.youtube.com/watch?v=2FYm3GOonhk)

-   [Camera Server Class](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/cameraserver/CameraServer.html#startAutomaticCapture())

-   [Open CV Java Documentation?(idk how modern this is) ](https://docs.opencv.org/4.x/javadoc/index.html)

-   Motors 

-   Encoders

-   Tasks Assigned (if any) 👍

* * * * *

Go Subteam + Tech Devils, We Got This!! >:) 

![](https://lh4.googleusercontent.com/--6ID6Rkfh9VvaFEvyAchcN5PwAjESddmXjjyLQ5aWv4eG_A38G-FQ3bS9nnD-12YE5up-VYDhHEj22BIOfQWr3iUku_DPC-ZN5Jn-7rrKNtNT-8EukaJMzSvFAAEgRqeezptGprDpu8OKqJ1dAQtno)