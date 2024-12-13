package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.PivotSubsystem.Positions;

public class GroundIntakeCommand extends NewtonWrapperCommand {
    public GroundIntakeCommand(){
        super(
            setPivotPositionCommand(Positions.GROUND) // Set the pivot to the ground position
            .alongWith(setRollerIntaking()) // intake the bucket
            .andThen(stopSubsystems(manager.getIntake())) // Stop the rollers once finished
        );
    }
}