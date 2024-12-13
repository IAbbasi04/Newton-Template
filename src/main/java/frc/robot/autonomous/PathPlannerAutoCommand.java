package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;

public abstract class PathPlannerAutoCommand extends AutoCommand {
    public PathPlannerAutoCommand(String fileName) {
        super(new PathPlannerAuto(fileName));
    }

    @Override
    public Pose2d getStartPose() {
        return PathPlannerAuto.getStaringPoseFromAutoFile(getName());
    }
}