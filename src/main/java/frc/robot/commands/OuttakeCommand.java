package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.proxies.NewtonWrapperCommand;

public class OuttakeCommand  extends NewtonWrapperCommand {
    public OuttakeCommand(){
        super(setRollerOuttaking()); // Spit out the bucket
    }
}
