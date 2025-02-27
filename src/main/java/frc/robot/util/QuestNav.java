package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class QuestNav {
    private boolean initializedPosition = false;
    private String networkTableRoot = "questnav";
    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable networkTable;
    private Transform3d robotToQuest;
    private Pose3d initPose = new Pose3d();
    private Transform3d softResetTransform = new Transform3d();
    private Pose3d softResetPose = new Pose3d();

    private IntegerEntry miso;
    private IntegerPublisher mosi;

    private IntegerSubscriber frameCount;
    private DoubleSubscriber timestamp;
    private FloatArraySubscriber position;
    private FloatArraySubscriber quaternion;
    private FloatArraySubscriber eulerAngles;
    private DoubleSubscriber battery;
    private double startTimestamp;

    private ChassisSpeeds velocity;
    private Pose3d previousPose;
    private double previousTime;
    private final double TIMESTAMP_DELAY = 0.002;

    private long previousFrameCount;

    private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
    private int _calculatedOffsetToRobotCenterCount = 0;

    public enum QuestCommand {
        RESET(1);

        public final int questRequestCode;

        private QuestCommand(int command) {
            this.questRequestCode = command;
        }

        public int getQuestRequest() {
            return questRequestCode;
        }
    }

    public QuestNav(Transform3d robotToQuest) {
        super();
        this.robotToQuest = robotToQuest;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    public QuestNav(Transform3d robotToQuest, String networkTableRoot) {
        super();
        this.robotToQuest = robotToQuest;
        this.networkTableRoot = networkTableRoot;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    private void setupInitialTimestamp() {
        startTimestamp = timestamp.get();
    }

    private void setupNetworkTables(String root) {
        networkTable = networkTableInstance.getTable(root);
        miso = networkTable.getIntegerTopic("miso").getEntry(0);
        mosi = networkTable.getIntegerTopic("mosi").publish();
        frameCount = networkTable.getIntegerTopic("frameCount").subscribe(0);
        timestamp = networkTable.getDoubleTopic("timestamp").subscribe(0.0);
        position = networkTable.getFloatArrayTopic("position").subscribe(new float[3]);
        quaternion = networkTable.getFloatArrayTopic("quaternion").subscribe(new float[4]);
        eulerAngles = networkTable.getFloatArrayTopic("eulerAngles").subscribe(new float[3]);
        battery = networkTable.getDoubleTopic("batteryPercent").subscribe(0.0); // Note: Changed to match your original code
    }

    public Translation3d getRawPosition() {
        return new Translation3d(position.get()[2], -position.get()[0], position.get()[1]);
    }

    private Translation3d rotateAxes(Translation3d raw, Rotation3d rotation) {
        return raw.rotateBy(rotation);
    }

    private Translation3d correctWorldAxis(Translation3d rawPosition) {
        return rotateAxes(rawPosition, robotToQuest.getRotation());
    }

    public Rotation3d getRawRotation() {
        float[] euler = eulerAngles.get();
        return new Rotation3d(Degrees.of(euler[2]), Degrees.of(euler[0]), Degrees.of(-euler[1]));
    }

    public Pose3d getRobotPose3d() {
        if (RobotBase.isReal()) {
            Pose3d pose = new Pose3d(getPosition(), getRotation());
            return pose;
        } else {
            return new Pose3d();
        }
    }
    
    public Pose2d getRobotPose() {
        return getRobotPose3d().toPose2d();
    }

    public Translation3d getProcessedPosition() {
        Translation3d correctedWorldAxis = correctWorldAxis(getRawPosition());
        Translation3d offsetCorrection = correctedWorldAxis
        .plus(robotToQuest.getTranslation())
        .plus(robotToQuest.getTranslation().times(-1).rotateBy(new Rotation3d(0, 0, getRawRotation().getZ())));
        Translation3d rotatedAxis = rotateAxes(offsetCorrection, initPose.getRotation());
        Translation3d hardResetTransformation = rotatedAxis.plus(initPose.getTranslation());
        return hardResetTransformation;
    }

    public Translation3d getPosition() {
        Translation3d hardResetTransform = getProcessedPosition();
        Translation3d softResetTransformation = rotateAxes(hardResetTransform.minus(softResetPose.getTranslation()), softResetTransform.getRotation()).plus(softResetPose.getTranslation()).plus(softResetTransform.getTranslation());
        return softResetTransformation;
    }

    public Rotation3d getProcessedRotation() {
        return getRawRotation().plus(initPose.getRotation());
    }

    public Rotation3d getRotation() {
       return getProcessedRotation().plus(softResetTransform.getRotation());
    }

    public double getTimestamp() {
        return timestamp.get();
    }

    public double getBatteryPercent() {
        return battery.get();
    }

    public boolean isConnected() {
        return ((RobotController.getFPGATime() - battery.getLastChange()) / 1000) < 250;
    }

    public boolean processQuestCommand(QuestCommand command) {
        if (miso.get() == 99) {
            return false;
        }
        mosi.set(command.getQuestRequest());
        return true;
    }

    private void resetQuestPose() {
        processQuestCommand(QuestCommand.RESET);
    }

    public void softReset(Pose3d pose) {
        softResetTransform = new Transform3d(pose.getTranslation().minus(getProcessedPosition()), pose.getRotation().minus(getProcessedRotation()));
        softResetPose = new Pose3d(getProcessedPosition(), getProcessedRotation());
    }

    public void hardReset(Pose3d pose) {
        initPose = pose;
        resetQuestPose();
    }

    // Reset the robot's pose on the field to match the provided pose
    public void resetPose(Pose2d newPose) {
        SmartDashboard.putBoolean("Reset Pose", true);
        initializedPosition = true;
        
        // Convert Pose2d to Pose3d for internal processing
        Pose3d pose3d = new Pose3d(
            newPose.getX(), 
            newPose.getY(), 
            0, 
            new Rotation3d(0, 0, newPose.getRotation().getRadians())
        );
        hardReset(pose3d);
    }

    public void cleanUpQuestCommand() {
        if (miso.get() == 99) {
            mosi.set(0);
        }
    }

    private void updateVelocity() {
        if (previousPose == null) {
            previousPose = getRobotPose3d();
            previousTime = timestamp.get();
            return;
        }
        double currentTime = timestamp.get();
        double deltaTime = currentTime - previousTime;
        if (deltaTime == 0) {
            return;
        }
        velocity = new ChassisSpeeds(
                (getPosition().getX() - previousPose.getTranslation().getX()) / deltaTime,
                (getPosition().getY() - previousPose.getTranslation().getY()) / deltaTime,
                (getRotation().getZ() - previousPose.getRotation().getZ()) / deltaTime);
        previousTime = currentTime;
        previousPose = getRobotPose3d();
    }

    public ChassisSpeeds getVelocity() {
        if (null != velocity) {
            return velocity;
        }
        return new ChassisSpeeds();
    }

    public void update() {
        if (RobotBase.isReal()) {
            cleanUpQuestCommand();
            updateVelocity();
            SmartDashboard.putBoolean("Reset Pose", false);

            Pose2d currPose = getRobotPose();
            SmartDashboard.putNumberArray("Quest POSE", new double[] {
                currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()
            });

            ChassisSpeeds velocity = getVelocity();
            SmartDashboard.putNumberArray("Quest Velocity", new double[] { 
                velocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond, 
                velocity.omegaRadiansPerSecond 
            });
        }
    }

    /**
     * Apply the calculated offset to the robot-to-Quest transform
     * @param offsetX X component of the offset
     * @param offsetY Y component of the offset
     * @return This QuestNav instance for method chaining
     */
    public QuestNav applyCalculatedOffset(double offsetX, double offsetY) {
        // Create a new transform that includes the offset
        robotToQuest = new Transform3d(
            new Translation3d(offsetX, offsetY, robotToQuest.getTranslation().getZ()),
            robotToQuest.getRotation()
        );
        
        return this;
    }

    /**
     * Get the current calculated offset to robot center
     * @return Translation2d with the current calculated offset
     */
    public Translation2d getCalculatedOffset() {
        return _calculatedOffsetToRobotCenter;
    }
    
    // Improve the calculateOffsetToRobotCenter method with better comments
    private Translation2d calculateOffsetToRobotCenter() {
        Pose3d currentPose = getRobotPose3d();
        Pose2d currentPose2d = currentPose.toPose2d();

        Rotation2d angle = currentPose2d.getRotation();
        Translation2d displacement = currentPose2d.getTranslation();

        // Add safety check to prevent division by very small numbers
        if (Math.abs(1 - angle.getCos()) < 1e-6) {
            return new Translation2d(); // Return zero offset when rotation is minimal
        }

        // Calculate the center of rotation based on the displacement and angle change
        // This uses the fact that when rotating around a point, the displacement follows
        // a circular path around that point
        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY()) / (2 * (1 - angle.getCos()));
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY()) / (2 * (1 - angle.getCos()));

        return new Translation2d(x, y);
    }

    public Command determineOffsetToRobotCenter(Drive drivetrain) {
        return Commands.repeatingSequence(
            Commands.run(
                () -> drivetrain.setControl(
                    new SwerveRequest.FieldCentric()
                    .withRotationalRate(0.314) // Slow rotation rate
                    .withVelocityX(0)
                    .withVelocityY(0)
                )
            ).withTimeout(0.5),
            Commands.runOnce(() -> {
                try {
                    // Update current offset
                    Translation2d offset = calculateOffsetToRobotCenter();
                    
                    // Average with previous values for robustness
                    _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter.times(
                        (double)_calculatedOffsetToRobotCenterCount / (_calculatedOffsetToRobotCenterCount + 1))
                        .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                    _calculatedOffsetToRobotCenterCount++;

                    SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] { 
                        _calculatedOffsetToRobotCenter.getX(), 
                        _calculatedOffsetToRobotCenter.getY() 
                    });
                } catch (Exception e) {
                    SmartDashboard.putString("Quest Calibration Error", e.getMessage());
                }
            }).onlyIf(() -> {
                try {
                    return getRotation().getZ() > Math.toRadians(30);
                } catch (Exception e) {
                    return false;
                }
            })
        );
    }
}

