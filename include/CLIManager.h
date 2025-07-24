#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <SimpleCLI.h>

SimpleCLI cli;
Command   cmdMotor;
Command   cmdOrgin;
Command   cmdStop;
Command   cmdRestart;
Command   cmdShow;
Command   cmdHelp;

void initializeCLI()
{
    cmdMotor = cli.addCmd("motor");
    cmdMotor.addArg("n", "1");    // motor number argument
    cmdMotor.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMotor.addArg("g", "0.0");  // positional argument (angle)
    cmdMotor.addFlagArg("c");     // current position
    cmdMotor.addFlagArg("d");     // disable flag
    cmdMotor.addFlagArg("e");     // enable flag
    cmdMotor.addFlagArg("j");     // closed-loop flag
    cmdMotor.addFlagArg("h");     // hybrid mode flag
    cmdMotor.setDescription("Control motor movement\r\n"
                            "Usage: motor -n <number> [-p <position>] [-c] [-d] [-e] [-j] [-h]\r\n"
                            "  -n: Motor number (1-4, required)\r\n"
                            "  -p: Target position (required for movement)\r\n"
                            "  -c: Get current position\r\n"
                            "  -d: Disable motor\r\n"
                            "  -e: Enable motor\r\n"
                            "  -j: Enable closed-loop control\r\n"
                            "  -h: Enable hybrid mode (read encoder once at start)\r\n"
                            "Examples:\r\n"
                            "  motor -n 1 -p 100.0    # Move motor 1 to 100 um (open-loop)\r\n"
                            "  motor -n 2 -p 45.0     # Move motor 2 to 45 degrees (open-loop)\r\n"
                            "  motor -n 1 -p 160.0 -j # Move motor 1 to 160 degrees (closed-loop)\r\n"
                            "  motor -n 1 -p 160.0 -h # Move motor 1 to 160 degrees (hybrid mode)\r\n"
                            "  motor -n 1 -c          # Get current position of motor 1\r\n"
                            "  motor -n 1 -d          # Disable motor 1\r\n"
                            "  motor -n 1 -e          # Enable motor 1\r\n");

    cmdOrgin = cli.addCmd("orgin");
    cmdOrgin.addArg("n", "1");  // motor number argument
    cmdOrgin.setDescription("Set origin position of motor");

    cmdStop = cli.addCmd("stop");
    cmdStop.setDescription("Stop the motor");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdShow = cli.addCmd("show");
    cmdShow.setDescription("Show the encoder and motor status");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");
}

#endif