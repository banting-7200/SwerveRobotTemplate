package frc.robot.Utilites;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldLayout {
    private AprilTagFieldLayout fieldLayout;

    public FieldLayout() {
        try {
            Path layoutPath = Filesystem.getDeployDirectory().toPath()
                    .resolve("fields/2025-reefscape-welded.json");
            fieldLayout = new AprilTagFieldLayout(layoutPath);
        } catch (IOException e) {
            System.out.println("Could not load Reefscape AprilTag layout");
        }

    }

    public Pose2d getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();

    }

    public Pose2d getOffsetedPosFromTag(int tagID) {
        Pose2d tagPose = getTagPose(tagID);

        Translation2d forwardOffset = new Translation2d(1.5, 0.0)
                .rotateBy(tagPose.getRotation());

        Rotation2d facingTag = tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

        Pose2d robotPose = new Pose2d(
                tagPose.getTranslation().plus(forwardOffset),
                facingTag);

        return robotPose;

    }

}
