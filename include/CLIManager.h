#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <SimpleCLI.h>

SimpleCLI cli;

Command cmdMotor;
Command cmdMove;
Command cmdMoveRelative;
Command cmdControlMode;
Command cmdStop;
Command cmdEnable;
Command cmdDisable;
Command cmdCurrentPosition;
Command cmdLastPosition;
Command cmdSave;
Command cmdRestart;
Command cmdShow;
Command cmdHelp;
Command cmdTest;

void initializeCLI()
{
    cmdMotor = cli.addCmd("motor");
    cmdMotor.addArg("n", "1");  // motor number argument
    cmdMotor.setDescription("Select the motor");

    cmdMove = cli.addCmd("move");
    cmdMove.addArg("n", "1");    // motor number argument
    cmdMove.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMove.setDescription("Move the current motor to the target position");

    cmdMoveRelative = cli.addCmd("mover");
    cmdMoveRelative.addArg("n", "1");    // motor number argument
    cmdMoveRelative.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMoveRelative.setDescription("Move the current motor relative to the current position");

    cmdControlMode = cli.addCmd("control");
    cmdControlMode.addArg("n", "1");  // motor number argument
    cmdControlMode.addFlagArg("o");   // open loop
    cmdControlMode.addFlagArg("h");   // hybrid
    cmdControlMode.addFlagArg("c");   // closed loop
    cmdControlMode.setDescription("Set control mode for current motor");

    cmdStop = cli.addCmd("stop");
    cmdStop.addArg("n", "1");  // motor number argument
    cmdStop.setDescription("Stop the current motor.");

    cmdEnable = cli.addCmd("enable");
    cmdEnable.addArg("n", "1");  // motor number argument
    cmdEnable.setDescription("Enable the current motor.");

    cmdDisable = cli.addCmd("disable");
    cmdDisable.addArg("n", "1");  // motor number argument
    cmdDisable.setDescription("Disable the current motor.");

    cmdCurrentPosition = cli.addCmd("position");
    cmdCurrentPosition.addArg("n", "1");  // motor number argument
    cmdCurrentPosition.setDescription("Show the current position of the current motor");

    cmdLastPosition = cli.addCmd("last");
    cmdLastPosition.addArg("n", "1");  // motor number argument
    cmdLastPosition.setDescription("Show the last position of the current motor");

    cmdSave = cli.addCmd("save");
    cmdSave.addArg("n", "1");  // motor number argument
    cmdSave.addFlagArg("o");   // orgin
    cmdSave.setDescription("Save the current position as origin of current motor");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdShow = cli.addCmd("show");
    cmdShow.addArg("n", "1");  // motor number argument
    cmdShow.setDescription("Show the encoder and motor status");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");

    cmdTest = cli.addCmd("test");
    cmdTest.addArg("p", "0.0");  // positional argument (um or deg)
    cmdTest.setDescription("Test the conversion functions");
}

#endif