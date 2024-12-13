package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;
import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.PivotSubsystem.Positions;

public class HumanPlayerIntakeCommand  extends NewtonWrapperCommand {
    public HumanPlayerIntakeCommand(){
        super(
            setPivotPositionCommand(Positions.HP_LOAD) // Set the pivot to the ground position
            .alongWith(setRollerIntaking()) // intake the bucket
            .andThen(stopSubsystems(manager.getIntake())) // Stop the rollers once finished
        );
    }
}
