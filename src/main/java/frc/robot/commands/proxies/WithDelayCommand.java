package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WithDelayCommand extends NewtonWrapperCommand {
    public WithDelayCommand(Command command, double delaySeconds) {
        super(new WaitCommand(delaySeconds).andThen(command));
    }
}
