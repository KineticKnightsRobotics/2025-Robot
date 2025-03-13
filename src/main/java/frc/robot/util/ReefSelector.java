package frc.robot.util;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefSelector {
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /*
    private final Pose2d[] blueAllianceReef = {
        tagLayout.getTagPose(17).get().toPose2d(),
        tagLayout.getTagPose(18).get().toPose2d(),
        tagLayout.getTagPose(19).get().toPose2d(),
        tagLayout.getTagPose(20).get().toPose2d(),
        tagLayout.getTagPose(21).get().toPose2d(),
        tagLayout.getTagPose(22).get().toPose2d()

    };

    private final Pose2d[] redAllianceReef = {
        tagLayout.getTagPose(6).get().toPose2d(),
        tagLayout.getTagPose(7).get().toPose2d(),
        tagLayout.getTagPose(8).get().toPose2d(),
        tagLayout.getTagPose(9).get().toPose2d(),
        tagLayout.getTagPose(10).get().toPose2d(),
        tagLayout.getTagPose(11).get().toPose2d()
    };
    */

    List<Pose2d> blueAllianceReef;
    List<Pose2d> redAllianceReef;

    public Boolean redAlliance;



    public ReefSelector() {
        redAlliance = false;

        blueAllianceReef.add(tagLayout.getTagPose(17).get().toPose2d());
        blueAllianceReef.add(tagLayout.getTagPose(18).get().toPose2d());
        blueAllianceReef.add(tagLayout.getTagPose(19).get().toPose2d());
        blueAllianceReef.add(tagLayout.getTagPose(20).get().toPose2d());
        blueAllianceReef.add(tagLayout.getTagPose(21).get().toPose2d());
        blueAllianceReef.add(tagLayout.getTagPose(22).get().toPose2d());

        redAllianceReef.add(tagLayout.getTagPose(6).get().toPose2d());
        redAllianceReef.add(tagLayout.getTagPose(7).get().toPose2d());
        redAllianceReef.add(tagLayout.getTagPose(8).get().toPose2d());
        redAllianceReef.add(tagLayout.getTagPose(9).get().toPose2d());
        redAllianceReef.add(tagLayout.getTagPose(10).get().toPose2d());
        redAllianceReef.add(tagLayout.getTagPose(11).get().toPose2d());

    }

    public Pose2d getClosestApriltag(Pose2d robotPose) {

        if (redAlliance) {
            return robotPose.nearest(redAllianceReef);
        }
        else {
            return robotPose.nearest(blueAllianceReef);
        }

    }


}
