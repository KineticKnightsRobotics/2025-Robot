package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class ReefSelector {
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /*
    private final Pose2d[] blueAllianceReefPoses = {
        tagLayout.getTagPose(17).get().toPose2d(),
        tagLayout.getTagPose(18).get().toPose2d(),
        tagLayout.getTagPose(19).get().toPose2d(),
        tagLayout.getTagPose(20).get().toPose2d(),
        tagLayout.getTagPose(21).get().toPose2d(),
        tagLayout.getTagPose(22).get().toPose2d()

    };

    private final Pose2d[] redAllianceReefPoses = {
        tagLayout.getTagPose(6).get().toPose2d(),
        tagLayout.getTagPose(7).get().toPose2d(),
        tagLayout.getTagPose(8).get().toPose2d(),
        tagLayout.getTagPose(9).get().toPose2d(),
        tagLayout.getTagPose(10).get().toPose2d(),
        tagLayout.getTagPose(11).get().toPose2d()
    };
    */

    List<Pose2d> blueAllianceReefPoses = new ArrayList<Pose2d>();
    int[] blueAllianceReefIDs = {17,18,19,20,21,22};
    List<Pose2d> redAllianceReefPoses = new ArrayList<Pose2d>();
    int[] redAllianceReefIDs =  {6 ,7 ,8 ,9 ,10,11};


    public Boolean redAlliance;



    public ReefSelector() {
        redAlliance = false;

        blueAllianceReefPoses.add(tagLayout.getTagPose(17).get().toPose2d());
        blueAllianceReefPoses.add(tagLayout.getTagPose(18).get().toPose2d());
        blueAllianceReefPoses.add(tagLayout.getTagPose(19).get().toPose2d());
        blueAllianceReefPoses.add(tagLayout.getTagPose(20).get().toPose2d());
        blueAllianceReefPoses.add(tagLayout.getTagPose(21).get().toPose2d());
        blueAllianceReefPoses.add(tagLayout.getTagPose(22).get().toPose2d());

        redAllianceReefPoses.add(tagLayout.getTagPose(6).get().toPose2d());
        redAllianceReefPoses.add(tagLayout.getTagPose(7).get().toPose2d());
        redAllianceReefPoses.add(tagLayout.getTagPose(8).get().toPose2d());
        redAllianceReefPoses.add(tagLayout.getTagPose(9).get().toPose2d());
        redAllianceReefPoses.add(tagLayout.getTagPose(10).get().toPose2d());
        redAllianceReefPoses.add(tagLayout.getTagPose(11).get().toPose2d());
    }

    public Pose2d getClosestApriltagPose(Pose2d robotPose) {
        if (redAlliance) {
            return robotPose.nearest(redAllianceReefPoses);
        }
        else {
            return robotPose.nearest(blueAllianceReefPoses);
        }
    }

    public Pose2d getBargePose() {
        if (redAlliance) {
            return tagLayout.getTagPose(5).get().toPose2d();
        }
        else {
            return tagLayout.getTagPose(14).get().toPose2d();
        }
    }

    public int getClosestApriltagID(Pose2d robotPose) {
        Pose2d pose;
        if (redAlliance) {
            pose = robotPose.nearest(redAllianceReefPoses);
            for (int x=0;x<=5;x++) {
                if (pose.equals(redAllianceReefPoses.get(x))) {
                    return redAllianceReefIDs[x];
                }
            }
            return 0;
        }
        else {
            pose = robotPose.nearest(blueAllianceReefPoses);
            for (int x=0;x<=5;x++) {
                if (pose.equals(blueAllianceReefPoses.get(x))) {
                    return blueAllianceReefIDs[x];
                }
            }
            return 0;
        }
    }
}
