package frc.robot.autonomous.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutoCommand;

public class DefaultAuto extends AutoCommand {
    @Override
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}