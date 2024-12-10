package frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public abstract class NewtonWrapperCommand extends WrapperCommand {
    public NewtonWrapperCommand(Command command){
        super(command);
        setName(getClass().getSimpleName());
    }

    public static Command stopSubsystems(NewtonSubsystem... toStop){
        Command result = Commands.none();
        for(NewtonSubsystem s:toStop){
            result = result.alongWith(s.runOnce(() -> {s.stop();}));
        }
        return result;
    }
}
