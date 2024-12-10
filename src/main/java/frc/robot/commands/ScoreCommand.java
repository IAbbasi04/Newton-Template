package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.proxies.NewtonWrapperCommand;

public class ScoreCommand extends NewtonWrapperCommand {
    public ScoreCommand() {
        super(
            // Ask for current pivot position
            // If we are at grid or low score position we use lower speed but we use higher speed for high score
            NewtonCommands.setIntakeVelocityCommand( // Set rollers to the intaking velocity
                Constants.INTAKE.TOP_ROLLER_SCORE_GRID_RPM, 
                Constants.INTAKE.BOTTOM_ROLLER_SCORE_GRID_RPM
            )
        );
    }
}
