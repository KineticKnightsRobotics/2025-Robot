package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.Vision;
import frc.robot.util.Quest;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.QuestNav;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Drive extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Vision kLimelight = new Vision("limelight-dignan", this);

    private QuestNav questNav;
    private boolean hasQuestInitialized = false;

    private AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Here's where you hardcode the Quest to Robot offset values
    private static final Transform3d QUEST_TO_ROBOT_TRANSFORM = new Transform3d(
        // ====================== CALIBRATION INSTRUCTIONS ======================
        // STEP 1: Run the robot and press the X button on driver controller
        // STEP 2: Observe "Quest Calculated Offset to Robot Center" in SmartDashboard
        // STEP 3: Replace these values with the X and Y offsets from Step 2
        // STEP 4: Redeploy code with your calibrated values
        // Example: If SmartDashboard shows [-0.123, 0.045], use those values below
        new Translation3d(-0.38, 0.000, 0),  // Calibrated values (in meters)
        new Rotation3d(0, 0, Math.toRadians(180))
    );
    
    public Command seedQuestPose() {
        return Commands.runOnce(() -> resetQuestPose());
    }

    private void resetQuestPose() {
        hasQuestInitialized = true;
        questNav.resetPose(getPose());
    }

    public Command disableQuest() {
        return Commands.runOnce(() -> hasQuestInitialized = false);
    }

    private void initializeQuestNav() {
        questNav = new QuestNav(QUEST_TO_ROBOT_TRANSFORM);
        SmartDashboard.putData("Questnav Seed Pose", seedQuestPose());
        SmartDashboard.putData("Questnav Disable", disableQuest());
    }

    //auto objects
    private Field2d field = new Field2d();
    private AutoBuilder autoBuilder;
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);

    // For storing trajectory history
    private final ArrayList<Pose2d> trajectoryHistory = new ArrayList<>();
    private final int MAX_TRAJECTORY_POINTS = 100;
    private int trajectoryUpdateCounter = 0;

    // Path Tracking fields
    private final Field2d mainField = field;  // Reuse existing field
    private final Field2d trajectoryField = new Field2d();
    private boolean followingTrajectory = false;
    private Pose2d currentTrajectoryPose = new Pose2d();
    private ChassisSpeeds commandedTrajectorySpeed = new ChassisSpeeds();
    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    // Add these to track velocities for plotting
    private double[] linearVelocityData = new double[2]; // [0]=commanded, [1]=actual
    private double[] angularVelocityData = new double[2]; // [0]=commanded, [1]=angular
    private double[] velocityErrorData = new double[2]; // [0]=linear error, [1]=angular error

    // For storing planned trajectory history
    private final ArrayList<Pose2d> plannedTrajectoryHistory = new ArrayList<>();

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Drive(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        ConfigureAutoBuilder();
        initializeQuestNav();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Drive(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        ConfigureAutoBuilder();
        initializeQuestNav();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Drive(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        ConfigureAutoBuilder();
        initializeQuestNav();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void ConfigureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getPose(),
                this::resetPose,
                () -> getState().Speeds,
                (speeds) -> {
                    // Store the commanded speeds for visualization
                    commandedTrajectorySpeed = speeds;
                    // Apply the control
                    this.setControl(autoRequest.withSpeeds(speeds));
                },
                new PPHolonomicDriveController(
                    new PIDConstants(3.0, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder, NOT SIGMA!!!!!!", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        // Apply operator perspective
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
         
        // Process Limelight vision
        SmartDashboard.putBoolean("Limelight TV", kLimelight.getTV());
        if (kLimelight.getTV() && !hasQuestInitialized) {
            addVisionMeasurement(
                kLimelight.getEstimatedRoboPose(),
                Utils.fpgaToCurrentTime(kLimelight.getTimestamp()),
                kLimelight.getStandardDeviations()
            );
        }

        // Update field visualization
        SmartDashboard.putData("field2d", this.field);
        field.getObject("PoseEstimatorPose").setPose(getPose());
        
        if (kLimelight.getTV()) {  
            Pose2d limelightPose = kLimelight.getEstimatedRoboPose();
            field.getObject("LimelightPose").setPose(limelightPose);
        }
        
        // Display pose data
        SmartDashboard.putNumberArray("Odometry Pose", new double[]{getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});
        SmartDashboard.putNumberArray("Limelight Pose", new double[]{kLimelight.getEstimatedRoboPose().getX(), kLimelight.getEstimatedRoboPose().getY(), kLimelight.getEstimatedRoboPose().getRotation().getDegrees()});
        
        // Process Quest data
        if (hasQuestInitialized) {
            addVisionMeasurement(
                questNav.getRobotPose(),
                Utils.fpgaToCurrentTime(questNav.getTimestamp()),
                VecBuilder.fill(0.1, 0.1, 0.1)
            );
        }

        questNav.cleanUpQuestCommand();

        // Display Quest data
        SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent());
        SmartDashboard.putBoolean("Quest Connected", questNav.isConnected());
        SmartDashboard.putBoolean("Quest Pose Seeded", hasQuestInitialized);
        
        // Display drive metrics
        SmartDashboard.putNumber("Robot Velocity", Units.inchesToMeters(this.getModule(0).getDriveMotor().getVelocity().getValueAsDouble()) / 6.75 * 4 * Math.PI);
        SmartDashboard.putNumber("Robot Acceleration", this.getModule(0).getDriveMotor().getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("Drive Current Draw", this.getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
        
        // Visualize Quest data
        if (questNav != null && questNav.isConnected()) {
            Pose2d questPose = questNav.getRobotPose();
            field.getObject("QuestPose").setPose(questPose);
            
            // Display Quest pose array
            SmartDashboard.putNumberArray("Quest Pose", 
                new double[] {
                    questPose.getX(),
                    questPose.getY(),
                    questPose.getRotation().getDegrees()
                });
        }

        // Update trajectory history
        if (++trajectoryUpdateCounter >= 5) {
            trajectoryUpdateCounter = 0;
            
            // Add current pose to history
            trajectoryHistory.add(getPose());
            
            // Limit history size
            while (trajectoryHistory.size() > MAX_TRAJECTORY_POINTS) {
                trajectoryHistory.remove(0);
            }
            
            // Update trajectory on Field2d
            field.getObject("RobotPath").setPoses(trajectoryHistory);
        }
        
        // Update velocity data
        if (followingTrajectory) {
            // Calculate linear velocity (magnitude of x and y components)
            double commandedLinearVel = Math.hypot(commandedTrajectorySpeed.vxMetersPerSecond, commandedTrajectorySpeed.vyMetersPerSecond);
            double actualLinearVel = Math.hypot(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond);
            
            // Store velocity data
            linearVelocityData[0] = commandedLinearVel;
            linearVelocityData[1] = actualLinearVel;
            angularVelocityData[0] = commandedTrajectorySpeed.omegaRadiansPerSecond;
            angularVelocityData[1] = getState().Speeds.omegaRadiansPerSecond;
            velocityErrorData[0] = commandedLinearVel - actualLinearVel;
            velocityErrorData[1] = commandedTrajectorySpeed.omegaRadiansPerSecond - getState().Speeds.omegaRadiansPerSecond;
        }

        if (followingTrajectory && currentTrajectoryPose != null) {
            // Add planned pose to history
            plannedTrajectoryHistory.add(currentTrajectoryPose);
            
            // Limit planned history size
            while (plannedTrajectoryHistory.size() > MAX_TRAJECTORY_POINTS) {
                plannedTrajectoryHistory.remove(0);
            }
            
            // Update planned path on trajectory field
            trajectoryField.getObject("Planned Path").setPoses(plannedTrajectoryHistory);
        }
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Pose2d getTagPose(int AprilTagID) {
        return kFieldLayout.getTagPose(AprilTagID).get().toPose2d();
    }

    public Translation2d getTranslationRelative(int apriltagID) { //Repurposed 2024 code
        return getPose().getTranslation().minus(
            kFieldLayout.getTagPose(apriltagID).get().getTranslation().toTranslation2d()
        );
    }
    
    // Getter methods for field and data
    public Field2d getField() {
        return field;
    }
    
    public Field2d getTrajectoryField() {
        return trajectoryField;
    }

    public double[] getLinearVelocityData() {
        return linearVelocityData;
    }

    public double[] getAngularVelocityData() {
        return angularVelocityData;
    }
    
    public QuestNav getQuestNav() {
        return questNav;
    }

    public double[] getVelocityErrorData() {
        return velocityErrorData;
    }

    /**
     * Apply calculated Quest offset to the main Quest instance
     * @param offsetX X component of the offset
     * @param offsetY Y component of the offset
     */
    public Command applyQuestCalibration(double offsetX, double offsetY) {
        return Commands.runOnce(() -> {
            questNav.applyCalculatedOffset(offsetX, offsetY);
            SmartDashboard.putString("Quest Calibration Status", 
                "Applied offset: [" + offsetX + ", " + offsetY + "]");
        });
    }

    /**
     * Apply calculated offset from a calibration Quest instance
     * @param calibrationQuest The QuestNav instance used for calibration
     */
    public Command applyQuestCalibration(QuestNav calibrationQuest) {
        return Commands.runOnce(() -> {
            if (calibrationQuest != null && questNav != null) {
                Translation2d offset = calibrationQuest.getCalculatedOffset();
                // The sign might need to be inverted depending on your coordinate system
                questNav.applyCalculatedOffset(offset.getX(), offset.getY());
                SmartDashboard.putString("Quest Calibration Status", 
                    "Applied offset: [" + offset.getX() + ", " + offset.getY() + "]");
            } else {
                SmartDashboard.putString("Quest Calibration Status", 
                    "Error: Quest instance not available");
            }
        });
    }

    public void setTrajectoryFollowing(boolean following) {
        followingTrajectory = following;
        if (!following) {
            // Clear the planned trajectory history when we stop following
            plannedTrajectoryHistory.clear();
        }
    }

    public boolean isFollowingTrajectory() {
        return followingTrajectory;
    }

    public ChassisSpeeds getCommandedChassisSpeeds() {
        return commandedTrajectorySpeed;
    }

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
}
