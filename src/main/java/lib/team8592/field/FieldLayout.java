package lib.team8592.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public abstract class FieldLayout {
    private AprilTagFieldLayout aprilTagLayout;
    public enum Alliance {
        kRed,
        kBlue;
    }

    public FieldLayout(AprilTagFieldLayout layout) {
        this.aprilTagLayout = layout;
    }

    public AprilTagFieldLayout getAprilTagLayout() {
        return this.aprilTagLayout;
    }
}