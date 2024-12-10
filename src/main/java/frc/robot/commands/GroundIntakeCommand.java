package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.proxies.NewtonWrapperCommand;

public class GroundIntakeCommand extends NewtonWrapperCommand {
    public GroundIntakeCommand() {
        super(
            // Set pivot position to the ground position and ...
            NewtonCommands.setIntakeVelocityCommand( // Set rollers to the intaking velocity
                Constants.INTAKE.TOP_ROLLER_INTAKE_RPM, 
                Constants.INTAKE.BOTTOM_ROLLER_INTAKE_RPM
            )
        );
    }
}
