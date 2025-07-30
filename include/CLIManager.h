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
    cmdMove.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMove.setDescription("Move the current motor to the target position");

    cmdMoveRelative = cli.addCmd("mover");
    cmdMoveRelative.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMoveRelative.setDescription("Move the current motor relative to the current position");

    cmdControlMode = cli.addCmd("control");
    cmdControlMode.addFlagArg("o");  // open loop
    cmdControlMode.addFlagArg("h");  // hybrid
    cmdControlMode.addFlagArg("c");  // closed loop
    cmdControlMode.setDescription("Set control mode for current motor");

    cmdStop = cli.addCmd("stop");
    cmdStop.setDescription("Stop the current motor.");

    cmdEnable = cli.addCmd("enable");
    cmdEnable.setDescription("Enable the current motor.");

    cmdDisable = cli.addCmd("disable");
    cmdDisable.setDescription("Disable the current motor.");

    cmdCurrentPosition = cli.addCmd("position");
    cmdCurrentPosition.setDescription("Show the current position of the current motor");

    cmdSave = cli.addCmd("save");
    cmdSave.addFlagArg("o");  // orgin
    cmdSave.setDescription("Save the current position as origin of current motor");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdShow = cli.addCmd("show");
    cmdShow.setDescription("Show the encoder and motor status");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");

    cmdTest = cli.addCmd("test");
    cmdTest.addArg("p", "0.0");  // positional argument (um or deg)
    cmdTest.setDescription("Test the conversion functions");
}

#endif