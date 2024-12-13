package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.PivotSubsystem.Positions;

public class PivotPrimeCommand extends NewtonWrapperCommand {
    public PivotPrimeCommand(Positions position) {
        super(
            // Set the pivot to the desired position indicated
            setPivotPositionCommand(position)
                .until(() -> manager.getPivot().atTargetPosition())
        );
    }
}