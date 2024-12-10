package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewtonSubsystem;

public abstract class LargeCommand extends Command {
    // Require at least one subsystem to be passed in
    public LargeCommand(NewtonSubsystem requirement1, NewtonSubsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
}