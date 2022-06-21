#include "iot_tui_node.h"

int main(int argc, char **argv) {

    // initialise ROS
    ros::init(argc, argv, "iot_tui");

    // warn Users about NodeHandle blocking when it can't connect to any master
    ROS_INFO("If no window appears this program probably can't connect to the ROS master");

    // NodeHandle is basically the entire ROS node stuff (publishers, subscribers, etc)
    ros::NodeHandle nh;

    // make sure ROS is working
    if (!ros::ok()) {
        ROS_FATAL("ROS died. There probably is a Node with the same name running.");
        exit(1);
    }


    // assign launch file values to launchParams, further elaboration in iot_tui.launch
    nh.getParam("/iot_tui_node/joystickDevice", launchParams.joystickDevice);
    nh.param("/iot_tui_node/mecanumMode", launchParams.mecanumMode, false);
    nh.param("/iot_tui_node/kb_enable", launchParams.kb_enable, true);
    nh.param("/iot_tui_node/kb_override", launchParams.kb_override, true);
    nh.param("/iot_tui_node/js_leftRight_axis", launchParams.js_leftRight_axis, 0);
    nh.param("/iot_tui_node/js_upDown_axis", launchParams.js_upDown_axis, 1);
    nh.param("/iot_tui_node/js_twist_axis", launchParams.js_twist_axis, 2);
    nh.param("/iot_tui_node/js_throttle_axis", launchParams.js_throttle_axis, 3);
    nh.param("/iot_tui_node/js_hatUpDown_axis", launchParams.js_hatUpDown_axis, 4);
    nh.param("/iot_tui_node/js_hatLeftRight_axis", launchParams.js_hatLeftRight_axis, 5);
    nh.param("/iot_tui_node/js_button_1", launchParams.js_buttons[0], 0);
    nh.param("/iot_tui_node/js_button_2", launchParams.js_buttons[1], 1);
    nh.param("/iot_tui_node/js_button_3", launchParams.js_buttons[2], 2);
    nh.param("/iot_tui_node/js_button_4", launchParams.js_buttons[3], 3);
    nh.param("/iot_tui_node/js_button_5", launchParams.js_buttons[4], 4);
    nh.param("/iot_tui_node/js_button_6", launchParams.js_buttons[5], 5);
    nh.param("/iot_tui_node/js_button_7", launchParams.js_buttons[6], 6);
    nh.param("/iot_tui_node/js_button_8", launchParams.js_buttons[7], 7);
    nh.param("/iot_tui_node/js_button_9", launchParams.js_buttons[8], 8);
    nh.param("/iot_tui_node/js_button_10", launchParams.js_buttons[9], 9);
    nh.param("/iot_tui_node/js_button_11", launchParams.js_buttons[10], 10);
    nh.param("/iot_tui_node/js_button_12", launchParams.js_buttons[11], 11);



    // TODO multiple joystick support. Startup TUI screen?

    // convert "string" joystickDevice to "const char *" and open file for reading
    int jsFile = open(launchParams.joystickDevice.c_str(), O_NONBLOCK);

    // warn user that there is no joystick
    if (jsFile == -1) ROS_WARN("Could not read joystick file! Is the joystick plugged in?");


    int ch; // store the character from getch()

    // lots of settings for ncurses
    initscr(); // initialise screen
    noecho(); // make getch() not print typed text
    cbreak(); // directly send typed characters to getch(), disable buffering
    start_color(); // makes coloring the terminal possible
    curs_set(0); // hides the cursor, which would otherwise get in the way
    clear(); // clear the screen
    refresh(); // refresh to apply settings
    keypad(stdscr, true); // enable keypad and arrow key support


//    quickie for converting button presses to integer values for ncurses

//    timeout(50);
//    int asd;
//    while(true){
//    asd = getch();
//    if(asd != -1){ printw("%i\n", asd);}
//    if(asd == 113){ endwin(); exit(0); }
//    }


    timeout(100); // timeout for the getch() -function

    // produce windows!
    rpmWindow = createRPM(0, 0);
    batTextWindow = createBattery(1, 22);
    if (launchParams.kb_enable) ctlKeyboardWindow = createKeyboardControls(10, 10); // only create variable if enabled
    joystickWindowStruct joystickWindows = createJoystick(10, 42);

    // produce subscribers and publishers!
    ros::Subscriber volt_sub = nh.subscribe("/voltage", 1, batteryCallback);
    ros::Subscriber rpm_sub = nh.subscribe("/rpm", 1, rpmCallback);
    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("/joy", 1);

    while (true) {
        ch = getch(); // read keyboard
        switch (ch) {

            //if q/Q is pressed, then exit
            case 113:
            case 81:
                endwin();
                exit(0);

            default:
                // if enabled, read and print keyboard values and change global values
                if (launchParams.kb_enable) printKeyboardControls(ctlKeyboardWindow, ch);
                getJoystickValues(jsFile); // read joystick and change global values
                publishJoystick(joy_pub); // publish joystick values
                printJoystick(joystickWindows); // print joystick values


                // do ROS Node magick once
                ros::spinOnce();

                // if ROS fails for whatever reason, actually shut down
                if (!ros::ok()) {
                    endwin();
                    ROS_FATAL("ROS died. There probably is a Node with the same name running.");
                    exit(1);
                }
                break;
        }
    }
}



// all RPM -related functions

WINDOW *createRPM(int urCornerY, int urCornerX) {

    // create Windows
    WINDOW *rpmFrameWin;
    rpmFrameWin = newwin(4, 21, urCornerY, urCornerX);

    WINDOW *rpmTextWin;
    rpmTextWin = newwin(2, 13, urCornerY + 1, urCornerX + 4);


    // Box and arrows
    box(rpmFrameWin, ACS_VLINE, ACS_HLINE);
    mvwprintw(rpmFrameWin, 0, 9, "^^^");

    // top left RPM
    mvwprintw(rpmFrameWin, 1, 1, "RPM");
    // top right RPM
    mvwprintw(rpmFrameWin, 1, 17, "RPM");
    // bottom left RPM
    mvwprintw(rpmFrameWin, 2, 1, "RPM");
    // bottom right RPM
    mvwprintw(rpmFrameWin, 2, 17, "RPM");

    // refresh window to show changes
    wrefresh(rpmFrameWin);

    return rpmTextWin;
}

void printRPM(WINDOW *rpmTextWin, int upL_rpmVal, int upR_rpmVal, int loL_rpmVal, int loR_rpmVal) {

    // clear old values
    wclear(rpmTextWin);

    //  ◆┬◆  top half of robot
    mvwaddch(rpmTextWin, 0, 5, ACS_DIAMOND);
    waddch(rpmTextWin, ACS_TTEE);
    waddch(rpmTextWin, ACS_DIAMOND);

    //  ◆┴◆  bottom half of robot
    mvwaddch(rpmTextWin, 1, 5, ACS_DIAMOND);
    waddch(rpmTextWin, ACS_BTEE);
    waddch(rpmTextWin, ACS_DIAMOND);


    // set top left RPM and value
    if (upL_rpmVal > -1) mvwprintw(rpmTextWin, 0, 1, "%i", upL_rpmVal);
    else mvwprintw(rpmTextWin, 0, 0, "%i", upL_rpmVal);

    // set top right RPM and value
    if (upR_rpmVal > -1) mvwprintw(rpmTextWin, 0, 9, "%i", upR_rpmVal);
    else mvwprintw(rpmTextWin, 0, 8, "%i", upR_rpmVal);

    // set bottom left RPM and value
    if (loL_rpmVal > -1) mvwprintw(rpmTextWin, 1, 1, "%i", loL_rpmVal);
    else mvwprintw(rpmTextWin, 1, 0, "%i", loL_rpmVal);

    // set bottom right RPM and value
    if (loR_rpmVal > -1) mvwprintw(rpmTextWin, 1, 9, "%i", loR_rpmVal);
    else mvwprintw(rpmTextWin, 1, 8, "%i", loR_rpmVal);

    // refresh window to show changes
    wrefresh(rpmTextWin);
}

void rpmCallback(const std_msgs::Float32MultiArray &msg) {

    // print the values from topic /rpm to the tui
    printRPM(
            rpmWindow,
            // since we don't need the precision of a float, convert the values to integers
            int(msg.data[0]),
            int(msg.data[1]),
            int(msg.data[2]),
            int(msg.data[3])
    );
}



// all battery -related functions

WINDOW *createBattery(int urCornerY, int urCornerX) {

    // create Windows
    WINDOW *batFrameWin;
    batFrameWin = newwin(3, 14, urCornerY, urCornerX);

    WINDOW *batTextWin;
    batTextWin = newwin(1, 9, urCornerY + 1, urCornerX + 1);


    // print top line of battery
    mvwaddch(batFrameWin, 0, 0, ACS_ULCORNER);
    for (int i = 1; i <= 9; i++) waddch(batFrameWin, ACS_HLINE);
    waddch(batFrameWin, ACS_URCORNER);
    wprintw(batFrameWin, "_");

    // print middle line of battery
    mvwaddch(batFrameWin, 1, 0, ACS_VLINE);
    mvwaddch(batFrameWin, 1, 10, ACS_VLINE);
    mvwaddch(batFrameWin, 1, 12, ACS_VLINE);

    // print bottom line of battery
    mvwaddch(batFrameWin, 2, 0, ACS_LLCORNER);
    for (int i = 1; i <= 9; i++) waddch(batFrameWin, ACS_HLINE);
    waddch(batFrameWin, ACS_LRCORNER);
    waddch(batFrameWin, ACS_S1);


    // refresh window to show changes
    wrefresh(batFrameWin);

    return batTextWin;
}

void printBattery(WINDOW *batTextWin, float voltage) {

    // initialise attribute inverted color (white background, black chars)
    init_pair(1, COLOR_BLACK, COLOR_WHITE);

    // if Battery voltage is higher than 24V the robot is charging (or the battery is about to explode. JK, BMS exists)
    if (voltage > 24) {

        // print charging on a white background
        wattron(batTextWin, COLOR_PAIR(1));
        mvwprintw(batTextWin, 0, 0, "charging ");
        wattroff(batTextWin, COLOR_PAIR(1));

        // refresh window to show changes
        wrefresh(batTextWin);

        return;
    }

    // map the raw Voltage input to percent
    // (to_high - to_low) * ((value - from_low) / (from_high - from_low))
    //(100 - 0) * ((voltage - 17) / (24 - 17))
    int charge = int(100 * ((voltage - 17) / 7));

    // make sure the battery does not go over 100% / under 0% due to imprecise measurement
    if (charge < 0) charge = 0;
    if (charge > 100) charge = 100;


    wattron(batTextWin, COLOR_PAIR(1));
    // print the color according to the percentage (fill level)
    switch (charge / 10) {

        case 10:
            mvwprintw(batTextWin, 0, 0, "%s%i%%%s", "  ", charge, "   ");
            wattroff(batTextWin, COLOR_PAIR(1));
            break;

        case 9:
            mvwprintw(batTextWin, 0, 0, "%s%i%%%s", "   ", charge, "   ");
            wattroff(batTextWin, COLOR_PAIR(1));
            break;

        case 8:
            mvwprintw(batTextWin, 0, 0, "%s%i%%%s", "   ", charge, "  ");
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, " ");
            break;

        case 7:
            mvwprintw(batTextWin, 0, 0, "%s%i%%%s", "   ", charge, " ");
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "  ");
            break;

        case 6:
            mvwprintw(batTextWin, 0, 0, "%s%i%%", "   ", charge);
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "   ");
            break;

        case 5:
            mvwprintw(batTextWin, 0, 0, "%s%i", "   ", charge);
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "%%   ");
            break;

        case 4:
            mvwprintw(batTextWin, 0, 0, "%s%i\b", "   ", charge);
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "%i%%   ", charge - 40);
            break;

        case 3:
            mvwprintw(batTextWin, 0, 0, "   ");
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "%i%%   ", charge);
            break;

        case 2:
            mvwprintw(batTextWin, 0, 0, "  ");
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, " %i%%   ", charge);
            break;

        case 1:
            mvwprintw(batTextWin, 0, 0, " ");
            wattroff(batTextWin, COLOR_PAIR(1));
            wprintw(batTextWin, "  %i%%   ", charge);
            break;

        case 0:
            wattroff(batTextWin, COLOR_PAIR(1));
            mvwprintw(batTextWin, 0, 0, "    %i%%   ", charge);
            break;
    }

    // refresh window to show changes
    wrefresh(batTextWin);
}

void batteryCallback(const std_msgs::Float32::ConstPtr &msg) {

    printBattery(batTextWindow, msg->data);
}



// the keyboard -related functions

WINDOW *createKeyboardControls(int urCornerY, int urCornerX) {

    // create Windows
    WINDOW *ctlFrameWin;
    ctlFrameWin = newwin(4, 7, urCornerY, urCornerX);

    WINDOW *ctlTextWin;
    ctlTextWin = newwin(2, 5, urCornerY + 1, urCornerX + 1);


    // print a box around the ctlFrameWin
    box(ctlFrameWin, ACS_VLINE, ACS_HLINE);

    // refresh the ctlFrameWin
    wrefresh(ctlFrameWin);

    // print the characters inside the ctlFrameWin
    mvwaddch(ctlTextWin, 0, 2, ACS_UARROW);
    mvwaddch(ctlTextWin, 0, 4, 101);
    mvwaddch(ctlTextWin, 1, 0, ACS_LARROW);
    mvwaddch(ctlTextWin, 1, 2, ACS_DARROW);
    mvwaddch(ctlTextWin, 1, 4, ACS_RARROW);

    // refresh the ctlTextWin
    wrefresh(ctlTextWin);

    return ctlTextWin;
}

void printKeyboardControls(WINDOW *ctlTextWin, int character) {
    // TODO add option to disable keyboard input

    // initialise attribute inverted color (white background, black chars)
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    wattron(ctlTextWin, COLOR_PAIR(1));

    peripheralValues.kb_isPressed = true;

    switch (character) {

        case 101: // e
        case 338: // page down
        case 339: // page up
            mvwaddch(ctlTextWin, 0, 4, 101);
            peripheralValues.kb_values[4] = true;
            if (peripheralValues.isEnabled) peripheralValues.isEnabled = false;
            else if (!peripheralValues.isEnabled) peripheralValues.isEnabled = true;
            break;

        case 119: // w
        case 259: // up arrow
            mvwaddch(ctlTextWin, 0, 2, ACS_UARROW);
            peripheralValues.kb_values[0] = true;
            break;

        case 97: // a
        case 260: // left arrow
            mvwaddch(ctlTextWin, 1, 0, ACS_LARROW);
            peripheralValues.kb_values[1] = true;
            break;

        case 115: // s
        case 258: // down arrow
            mvwaddch(ctlTextWin, 1, 2, ACS_DARROW);
            peripheralValues.kb_values[2] = true;
            break;

        case 100: // d
        case 261: // right arrow
            mvwaddch(ctlTextWin, 1, 4, ACS_RARROW);
            peripheralValues.kb_values[3] = true;
            break;


        default:
            wattroff(ctlTextWin, COLOR_PAIR(1));
            mvwaddch(ctlTextWin, 0, 2, ACS_UARROW);
            mvwaddch(ctlTextWin, 0, 4, 101);
            mvwaddch(ctlTextWin, 1, 0, ACS_LARROW);
            mvwaddch(ctlTextWin, 1, 2, ACS_DARROW);
            mvwaddch(ctlTextWin, 1, 4, ACS_RARROW);
            peripheralValues.kb_values[0] = false;
            peripheralValues.kb_values[1] = false;
            peripheralValues.kb_values[2] = false;
            peripheralValues.kb_values[3] = false;
            peripheralValues.kb_values[4] = false;
            peripheralValues.kb_isPressed = false;
    }

    wattroff(ctlTextWin, COLOR_PAIR(1));

    wrefresh(ctlTextWin);

}



// all joystick -related functions

void publishJoystick(const ros::Publisher &joystick_pub) {

    // struct for ROS joystick publisher
    sensor_msgs::Joy rosJoyValues = sensor_msgs::Joy();

    // set the header's values
    rosJoyValues.header.stamp = ros::Time::now(); // set header time to current time
    rosJoyValues.header.frame_id = "joy";


    // fill arrays with -0.0 / 0. Only "new" values are transmitted, so we have to add our own 0's
    rosJoyValues.axes.assign(6, -0.0);
    rosJoyValues.buttons.assign(12, 0);

    // if Keyboard is enabled, pressed and kb_override is true
    if (launchParams.kb_enable && peripheralValues.kb_isPressed && launchParams.kb_override) {
        // publish keyboard values

        // insert local values to rosJoyValues
        if (peripheralValues.kb_values[0]) rosJoyValues.axes[1] = 1.0;
        if (peripheralValues.kb_values[1]) rosJoyValues.axes[0] = 1.0;
        if (peripheralValues.kb_values[2]) rosJoyValues.axes[1] = -1.0;
        if (peripheralValues.kb_values[3]) rosJoyValues.axes[0] = -1.0;

        // as long as peripheralValues.isEnabled is true, keep telling the robot to enable, if false disable robot
        if (peripheralValues.isEnabled) rosJoyValues.buttons[10] = 1;
        else if (!peripheralValues.isEnabled) rosJoyValues.buttons[9] = 1;


    } else {
        // publish joystick values

        // map all js_axisValues to -1 ─ 1 and insert them
        for (int i = 0; i < 6; i++) rosJoyValues.axes[i] = (peripheralValues.js_axisValues[i] / -32767);

        // insert all js_buttonValues into rosJoyValues
        for (int i = 0; i < 12; i++) rosJoyValues.buttons[i] = peripheralValues.js_buttonValues[i];

        // as long as peripheralValues.isEnabled is true, keep telling the robot to enable, if false disable robot
        if (peripheralValues.isEnabled) rosJoyValues.buttons[10] = 1;
        else if (!peripheralValues.isEnabled) rosJoyValues.buttons[9] = 1;


    }
    // actually publish values
    joystick_pub.publish(rosJoyValues);
}

joystickWindowStruct createJoystick(int urCornerY, int urCornerX) {

    joystickWindowStruct joystickWins{};

    // create Window
    WINDOW *joystickFrameWin;
    joystickFrameWin = newwin(9, 25, urCornerY, urCornerX);

    // print a box around the joystickFrameWin
    box(joystickFrameWin, ACS_VLINE, ACS_HLINE);

    // vertical line
    mvwvline(joystickFrameWin, 1, 12, ACS_VLINE, 7); // print the row of "│" in the middle column
    mvwaddch(joystickFrameWin, 0, 12, ACS_TTEE); // print "┬" in the middle of the 1st line
    mvwaddch(joystickFrameWin, 8, 12, ACS_BTEE); // print "┴" in the middle of the 9th line

    // horizontal line
    mvwhline(joystickFrameWin, 6, 1, ACS_HLINE, 23); // print the row of "─" in the 6th line
    mvwaddch(joystickFrameWin, 6, 0, ACS_LTEE); // print "├" at the start of the 6th line
    mvwaddch(joystickFrameWin, 6, 24, ACS_RTEE); // print "┤" ath the end of the 6th line

    // the intersection
    mvwaddch(joystickFrameWin, 6, 12, ACS_PLUS); // print "┼" in the middle of the 6th line

    // refresh the ctlFrameWin
    wrefresh(joystickFrameWin);



    // assign windows to the joystickWins struct
    joystickWins.axis_UL_win = newwin(5, 11, urCornerY + 1, urCornerX + 1);
    joystickWins.axis_UR_win = newwin(5, 11, urCornerY + 1, urCornerX + 13);
    joystickWins.axis_LL_win = newwin(1, 11, urCornerY + 7, urCornerX + 1);
    joystickWins.axis_LR_win = newwin(1, 11, urCornerY + 7, urCornerX + 13);


    // print the "+" in axis_UR_win
    mvwaddch(joystickWins.axis_UL_win, 2, 5, 43);
    // refresh the axis_UR_win
    wrefresh(joystickWins.axis_UL_win);

    // print the "+" in axis_UL_win
    mvwaddch(joystickWins.axis_UR_win, 2, 5, 43);
    // refresh the axis_UL_win
    wrefresh(joystickWins.axis_UR_win);

    // print the "|" in axis_LR_win
    mvwaddch(joystickWins.axis_LL_win, 0, 5, 124);
    // refresh the axis_LR_win
    wrefresh(joystickWins.axis_LL_win);

    // print the "|" in axis_LL_win
    mvwaddch(joystickWins.axis_LR_win, 0, 5, 124);
    // refresh the axis_LL_win
    wrefresh(joystickWins.axis_LR_win);

    return joystickWins;
}

void printJoystick(joystickWindowStruct joystickWins) {

    int percentages[6];

    // if Keyboard is enabled, pressed and kb_override is true
    if (launchParams.kb_enable && peripheralValues.kb_isPressed && launchParams.kb_override) {

        // use keyboard values

        percentages[0] = 20;
        percentages[1] = 20;
        if (peripheralValues.kb_values[3]) percentages[0] = 40;
        if (peripheralValues.kb_values[1]) percentages[0] = 0;
        if (peripheralValues.kb_values[2]) percentages[1] = 40;
        if (peripheralValues.kb_values[0]) percentages[1] = 0;
        percentages[2] = 20;
        percentages[3] = 20;
        percentages[4] = 20;
        percentages[5] = 20;

    } else {

        // use joystick values

        for (int i = 0; i <= 5; i++)
            // map from -32767 ─ 32767 to 0 ─ 40, as 40 can be divided through 4 and 10, thus directly converted into window coordinates
            // (to_high - to_low) * ((value - from_low) / (from_high - from_low))
            //(40 - 0) * ((peripheralValues.js_axisValues[i] - -32767) / (32767 - -32767))
            percentages[i] = int(40 * ((peripheralValues.js_axisValues[i] + 32767) / 65534));
    }

    // clear the axis_UL_win, then print values, then refresh
    wclear(joystickWins.axis_UL_win);

    // TODO add buttons!

    // create a bunch of different color pairs
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_RED, COLOR_WHITE);

    if (peripheralValues.js_buttonValues[1] || peripheralValues.kb_values[4]) {
        if (peripheralValues.isEnabled) {
            wattron(joystickWins.axis_UL_win, COLOR_PAIR(3));
            mvwaddch(joystickWins.axis_UL_win, 0, 0, 101);
            wattroff(joystickWins.axis_UL_win, COLOR_PAIR(3));
        } else {
            wattron(joystickWins.axis_UL_win, COLOR_PAIR(1));
            mvwaddch(joystickWins.axis_UL_win, 0, 0, 101);
            wattroff(joystickWins.axis_UL_win, COLOR_PAIR(1));
        }
    } else {
        if (peripheralValues.isEnabled) {
            wattron(joystickWins.axis_UL_win, COLOR_PAIR(2));
            mvwaddch(joystickWins.axis_UL_win, 0, 0, 101);
            wattroff(joystickWins.axis_UL_win, COLOR_PAIR(2));
        } else {
            mvwaddch(joystickWins.axis_UL_win, 0, 0, 101);
        }
    }

    mvwaddch(joystickWins.axis_UL_win, 2, 5, 43); // add "+" in the center
    mvwaddch(joystickWins.axis_UL_win, percentages[1] / 10, percentages[0] / 4, ACS_CKBOARD);
    wrefresh(joystickWins.axis_UL_win);

    // clear the axis_UR_win, then print values, then refresh
    wclear(joystickWins.axis_UR_win);
    mvwaddch(joystickWins.axis_UR_win, 2, 5, 43); // add "+" in the center
    mvwaddch(joystickWins.axis_UR_win, percentages[5] / 10, percentages[4] / 4, ACS_CKBOARD);
    wrefresh(joystickWins.axis_UR_win);

    // clear the axis_LL_win, then print values, then refresh
    wclear(joystickWins.axis_LL_win);
    mvwaddch(joystickWins.axis_LL_win, 0, 5, 124); // add "|" in the middle
    mvwaddch(joystickWins.axis_LL_win, 0, percentages[2] / 4, ACS_CKBOARD);
    wrefresh(joystickWins.axis_LL_win);

    // clear the axis_LR_win, then print values, then refresh
    wclear(joystickWins.axis_LR_win);
    mvwaddch(joystickWins.axis_LR_win, 0, 5, 124); // add "|" in the middle
    mvwaddch(joystickWins.axis_LR_win, 0, percentages[3] / 4, ACS_CKBOARD);
    wrefresh(joystickWins.axis_LR_win);

}

void getJoystickValues(int jsFile) {

    // if jsFile is "-1", then no joystick file was found
    if (jsFile == -1) return;


    // initialise the linux joystick driver struct
    struct js_event joystickEvent{};


    // read the joystick and parse data
    while (read(jsFile, &joystickEvent, sizeof(joystickEvent)) > 1) {

        switch (joystickEvent.type) {

            case 0x01: // button pressed/released
                // apply remaps to buttons
                peripheralValues.js_buttonValues[launchParams.js_buttons[joystickEvent.number]] = joystickEvent.value;
                if (joystickEvent.number == 1) peripheralValues.isEnabled = true;
                if (joystickEvent.number == 3) peripheralValues.isEnabled = false;
                break;

            case 0x02: // axis moved

                switch (joystickEvent.number) { // assign and remap incoming values to corresponding peripheralValues js_axis

                    case 0:
                        peripheralValues.js_axisValues[launchParams.js_leftRight_axis] = joystickEvent.value;
                        break;

                    case 1:
                        peripheralValues.js_axisValues[launchParams.js_upDown_axis] = joystickEvent.value;
                        break;

                    case 2:
                        peripheralValues.js_axisValues[launchParams.js_twist_axis] = joystickEvent.value;
                        break;

                    case 3:
                        peripheralValues.js_axisValues[launchParams.js_throttle_axis] = joystickEvent.value;
                        break;

                    case 4:
                        peripheralValues.js_axisValues[launchParams.js_hatLeftRight_axis] = joystickEvent.value;
                        break;

                    case 5:
                        peripheralValues.js_axisValues[launchParams.js_hatUpDown_axis] = joystickEvent.value;
                        break;
                }
                break;
        }
    }
}
