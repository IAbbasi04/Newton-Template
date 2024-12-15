package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DelayWrapperCommand extends NewtonWrapperCommand {
    public DelayWrapperCommand(Command command, double delaySeconds) {
        super(new WaitCommand(delaySeconds).andThen(command));
    }
}
