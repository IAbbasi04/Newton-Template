package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.PivotSubsystem.Positions;

public class ScoreLowCommand extends NewtonWrapperCommand {
    public ScoreLowCommand(){
        super(
            setPivotPositionCommand(Positions.GROUND) // Set the pivot to the high score position
            .until(() -> manager.getPivot().atTargetPosition())
            .andThen(
                setRollerScoring(), // Score the bucket once we are at position
                stopSubsystems(manager.getIntake()) // Stop the rollers once we've scored
            )
        );
    }
}