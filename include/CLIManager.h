#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <SimpleCLI.h>

SimpleCLI cli;
Command   cmdMotor;
/*
Command   cmdRestart;
Command   cmdStop;
Command   cmdShow;
Command   cmdDrive;
Command   cmdMinSpeedDetect;
Command   cmdReset;
Command   cmdHelp;
*/

void initializeCLI()
{
    cmdMotor = cli.addCmd("otor");
    cmdMotor.addArg("i", "1");    // motor number argument
    cmdMotor.addArg("p", "0.0");  // positional argument (um or deg)

    /*
cmdMotor.addFlagArg("c");   // current position
cmdMotor.addFlagArg("d");   // disable flag
cmdMotor.addFlagArg("e");   // enable flag

cmdMotor.setDescription("Control motor movement\r\n"
"Usage: motor -n <number> [-p <position>] [-c] [-d]\r\n"
"  -n: Motor number (1-4, required)\r\n"
"  -p: Target position (required for movement)\r\n"
"  -c: Get current position\r\n"
"  -d: Disable motor\r\n"
"Examples:\r\n"
"  motor -n 1 -p 100.0    # Move motor 1 to 100 um\r\n"
"  motor -n 2 -p 45.0     # Move motor 2 to 45 degrees\r\n"
"  motor -n 1 -c          # Get current position of motor 1\r\n"
"  motor -n 1 -d          # Disable motor 1\r\n"
"  motor -n 1 -e          # Enable motor 1\r\n");

cmdRestart = cli.addCmd("restart");
cmdRestart.setDescription("Restart the ESP32 system");

cmdStop = cli.addCmd("stop");
cmdStop.setDescription("Stop the motor");

cmdShow = cli.addCmd("show");
cmdShow.setDescription("Show the encoder and motor status");

cmdDrive = cli.addCmd("drv");
cmdDrive.setDescription("Show the drive status");

cmdMinSpeedDetect = cli.addCmd("speed");
cmdMinSpeedDetect.setDescription("Demonstrate the stepped speed profile");

cmdReset = cli.addCmd("reset");
cmdReset.setDescription("Reset the motor");

cmdHelp = cli.addCmd("help");
cmdHelp.setDescription("Show help information");
*/
}

#endif