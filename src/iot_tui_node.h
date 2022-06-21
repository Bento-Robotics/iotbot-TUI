#ifndef IOT_TUI_NODE_H
#define IOT_TUI_NODE_H


#include <ncurses.h> // ncurses
#include <ros/ros.h> // ROS
#include <std_msgs/Float32.h> // for ROS float values
#include <std_msgs/Float32MultiArray.h> // for ROS float arrays
#include <sensor_msgs/Joy.h> // for ROS joystick message
#include <linux/joystick.h> // for reading out joystick
#include <fcntl.h> // for nonblock reading

// struct for the .launch file
struct launchParams {

    std::string joystickDevice; // string for joystick file, change value in iot_tui.launch if needed

    // If mecanumMode is true, steering will be the js_twist_axis,
    // and driving diagonally will be the js_leftRight_axis.
    //
    // If it is false, steering will be the js_leftRight_axis and
    // the js_twist_axis will be ignored.
    bool mecanumMode{};
    bool kb_enable{}; // enable/disable keyboard controls. Will not display keyboardControls Window if disabled.
    bool kb_override{}; // If kb_override is true, keyboard input will override Joystick input. Vice versa if false.

    int js_leftRight_axis{}; // change values in iot_tui.launch to remap joystick
    int js_upDown_axis{}; // change values in iot_tui.launch to remap joystick
    int js_twist_axis{}; // change values in iot_tui.launch to remap joystick
    int js_throttle_axis{}; // change values in iot_tui.launch to remap joystick
    int js_hatLeftRight_axis{}; // change values in iot_tui.launch to remap joystick
    int js_hatUpDown_axis{}; // change values in iot_tui.launch to remap joystick
    int js_buttons[12]{}; // change values in iot_tui.launch to remap joystick

};


// declare struct (as datatype) to save the joystick windows
struct joystickWindowStruct {
    WINDOW *axis_UR_win;
    WINDOW *axis_UL_win;
    WINDOW *axis_LR_win;
    WINDOW *axis_LL_win;
};


// TODO find a less restrictive way of storing keyboard values ↓↓↓

// declare struct peripheralValues to save peripheral values
struct peripheralValues {
    float js_axisValues[6]; // array for joystick axis values
    bool js_buttonValues[12]; // array for joystick button values

    bool isEnabled; // bool to signal when the robot should be enabled

    // array for keyboard values
    // 0=w, 1=a, 2=s, 3=d, 4=enable
    bool kb_values[5];
    bool kb_isPressed; // bool to signal if any buttons are pressed
} peripheralValues; // and initialise it


//initialise global variables
launchParams launchParams;
WINDOW *rpmWindow;
WINDOW *batTextWindow;
WINDOW *ctlKeyboardWindow;


/*
 * Takes two integers (urCornerY, urCornerX) and uses these for
 * the coordinates of the top right corner of the window.
 *
 * Creates two windows (rpmFrameWin, rpmTextWin),
 * prints a box and "RPM" into rpmFrameWin, refreshes rpmFrameWin
 * then returns rpmTextWin for printRPM().
 *
 * Layout:
 * ┌────────^^^────────┐
 * │RPM             RPM│
 * │RPM             RPM│
 * └───────────────────┘
 */
WINDOW *createRPM(int, int);

/*
 * Takes a Window (rpmTextWin) to print the 4 taken integers to.
 * (Names: e.g. loL = lower Left )
 *
 * Prints integers and a robot to rpmTextWin, then refreshes it.
 *
 * Layout:
 * " 1   ◆┬◆-1   "
 * "-999 ◆┴◆-999 "
 */
void printRPM(WINDOW *, int, int, int, int);

// callback function to get RPM values from ROS and call printRPM() with them
void rpmCallback(const std_msgs::Float32MultiArray &msg);


/*
 * Creates the battery window & frame, then returns said window.
 * ┌───────────┐_
 * │           │ │
 * └───────────┘⎺
 * Use the printBattery() function to add a "fill level".
 */
WINDOW *createBattery(int, int);

/*
 * Takes a Window (batTextWin) to print the taken float to.
 * (range 30-17)
 *
 * Maps form 0 to 100, then prints float to rpmTextWin and
 * formats it to add a "fill level" to rpmTextWin, then refreshes it.
 *
 * Layout:
 * "▒▒ 23%   "
 */
void printBattery(WINDOW *, float);

// callback function to get battery Voltage from ROS and call printBattery() with them
void batteryCallback(const std_msgs::Float32::ConstPtr &);


/*
 * Creates the controls window & frame, adds text graphics, then returns said window.
 * ┌─────┐
 * │  ^ e│
 * │< v >│
 * └─────┘
 * Use the printControls() function for keyboard input.
 */
WINDOW *createKeyboardControls(int, int);

/*
 * reads the keyboard and highlights characters from createKeyboardControls() when they are pressed.
 * also sets values in global struct peripheralValues according to key presses
 */
void printKeyboardControls(WINDOW *, int);


/*
 * Creates the controls window & frame, then returns said window.
 * ┌───────────┬───────────┐
 * │           │           │
 * │           │           │
 * │     +     │     +     │
 * │           │           │
 * │           │           │
 * ├───────────┼───────────┤
 * │     |     │     |     │
 * └───────────┴───────────┘
 *
 *
 */
joystickWindowStruct createJoystick(int, int);

/*
 * Prints "▒" in the windows supplied by joystickWindowStruct,
 * according to the values in global struct peripheralValues
 * ┌───────────┬───────────┐
 * │           │           │
 * │           │           │
 * │     +     │     ▒     │
 * │           │           │
 * │   ▒       │           │
 * ├───────────┼───────────┤
 * │     ▒     │   ▒ |     │
 * └───────────┴───────────┘
 * if the "▒"- cursor is not centered, a "+" (or a "|") will be printed in the center
 *
 * Layout axis values:
 *      1x0 │ 5x4
 *      ────┼────
 *        2 │ 3
 */
void printJoystick(joystickWindowStruct);

/*
 * getJoystickValues() is designed to read all queued joystick-value-change events from the Linux driver,
 * then piece together the overall values of the device. Insert the same variable that returns out,
 * and the values will be updated.
 *
 * This is necessary because the Linux joystick driver returns singular value changes, and not all current
 * values of the device, we must piece together the values of the device as they come in, speak we need to save them.
 */
void getJoystickValues(int);

// parse, remap and publish values from peripheralValues to ros::Publisher
void publishJoystick(const ros::Publisher &);


#endif //IOT_TUI_NODE_H
