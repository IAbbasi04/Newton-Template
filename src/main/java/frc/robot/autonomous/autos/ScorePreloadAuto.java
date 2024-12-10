package frc.robot.autonomous.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutoCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.ScoreCommand;

public class ScorePreloadAuto extends AutoCommand {
    public ScorePreloadAuto() {
        super(
              new ScoreCommand().withTimeout(0.5).andThen(
                new GroundIntakeCommand().withTimeout(1.0)
              )
        );
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}
