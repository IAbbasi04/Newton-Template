package frc.robot.autonomous.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoCommand;
import frc.robot.commands.proxies.DelayWrapperCommand;

public class BasicRandomAuto extends AutoCommand {
    public BasicRandomAuto() {
        super(
            new DelayWrapperCommand(
                new WaitCommand(1.0), 
                1.0    
            )
        );
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(new Translation2d(3, 1), new Rotation2d());
    }
}