package frc.robot.commands;

import static frc.robot.commands.NewtonCommands.*;

import frc.robot.Constants;
import frc.robot.commands.proxies.NewtonWrapperCommand;
import frc.robot.subsystems.PivotSubsystem.Positions;

public class ScoreHighCommand extends NewtonWrapperCommand {
    public ScoreHighCommand(){
        super(
            setPivotPositionCommand(Positions.SCORE_HIGH) // Set the pivot to the high score position
            .until(() -> manager.getPivot().atTargetPosition())
            .andThen(
                setRollerScoring().withTimeout(Constants.INTAKE.SCORE_TIME), // Score the bucket once we are at position
                manager.getIntake().getStopCommand() // Stop the rollers once we've scored
            )
        );
    }
}
