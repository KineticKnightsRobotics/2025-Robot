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

    private static final Transform3d QUEST_TO_ROBOT_TRANSFORM = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, Math.toRadians(180))
    );
    
    private void initializeQuestNav() {
    private void initializeQuestNav() {
        questNav = new QuestNav(QUEST_TO_ROBOT_TRANSFORM);
        SmartDashboard.putData("Questnav Seed Pose", seedQuestPose());
        SmartDashboard.putData("Questnav Disable", disableQuest());
    }

    //auto objects
    private Field2d field = new Field2d();
    private AutoBuilder autoBuilder;
        .withDriveRequestType(DriveRequestType.Velocity);

    // For storing trajectory history
    private final ArrayList<Pose2d> trajectoryHistory = new ArrayList<>();
    private final int MAX_TRAJECTORY_POINTS = 100;
    private int trajectoryUpdateCounter = 0;


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * <p>
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
     * <p>
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
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
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

        questNav = new QuestNav(new Transform3d(
            new Translation3d(0, 0, 0), // Position offset (x, y, z)
            new Rotation3d(0, 0, Math.toRadians(180)) // Orientation offset
        ));

        SmartDashboard.putData("Questnav Seed Pose", seedQuestPose());
        SmartDashboard.putData("Questnav Disable", disableQuest());
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

    public void ConfigureAutoBuilder(){
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                ()->getPose(), //Taken from CTRE Examples: https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java#L197
                this::resetPose,
                ()->getState().Speeds,
                (speeds)-> this.setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(3.0, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder, NOT SIGMA!!!!!!", ex.getStackTrace());
        }
    }



    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
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
         
        SmartDashboard.putBoolean("Limelight TV", kLimelight.getTV());
        if (kLimelight.getTV() && !hasQuestInitialized) {
            //setVisionMeasurementStdDevs(kLimelight.getStandardDeviations());
            addVisionMeasurement(
                kLimelight.getEstimatedRoboPose(),
                Utils.fpgaToCurrentTime(kLimelight.getTimestamp()),//kLimelight.getTimestamp(),
                kLimelight.getStandardDeviations()
            );
        }

        SmartDashboard.putData("field2d", this.field);

       // this.field.setRobotPose(getPose());
       field.getObject("PoseEstimatorPose").setPose(getPose());

       if (kLimelight.getTV()) {  
            Pose2d limelightPose = kLimelight.getEstimatedRoboPose();
            field.getObject("LimelightPose").setPose(limelightPose);
        }
        
        SmartDashboard.putNumberArray("Odometry Pose", new double[]{getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});
        SmartDashboard.putNumberArray("Limelight Pose", new double[]{kLimelight.getEstimatedRoboPose().getX(), kLimelight.getEstimatedRoboPose().getY(), kLimelight.getEstimatedRoboPose().getRotation().getDegrees()});
        
        

        if (hasQuestInitialized) {
            addVisionMeasurement(
                questNav.getRobotPose(),
                Utils.fpgaToCurrentTime(questNav.getTimestamp()),
                VecBuilder.fill(0.1, 0.1, 0.1)
            );
        }

        questNav.cleanUpQuestCommand();

        SmartDashboard.putNumber("Quest Battery",questNav.getBatteryPercent());
        SmartDashboard.putBoolean("Quest Connected", questNav.isConnected());
        SmartDashboard.putBoolean("Quest Pose Seeded", hasQuestInitialized);
        double[] questPoseArray = {questNav.getRobotPose().getX(),questNav.getRobotPose().getY()};
        SmartDashboard.putNumberArray("Quest Pose", questPoseArray);

        SmartDashboard.putNumber("Robot Velocity", Units.inchesToMeters(this.getModule(0).getDriveMotor().getVelocity().getValueAsDouble()) / 6.75 * 4 * Math.PI);
        SmartDashboard.putNumber("Robot Accelleration", this.getModule(0).getDriveMotor().getAcceleration().getValueAsDouble());

        SmartDashboard.putNumber("Drive Curerent Draw",this.getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
        
        // Improve Quest data visualization
        if (questNav != null) {
            if (questNav.isConnected()) {
                Pose2d questPose = questNav.getRobotPose();
                field.getObject("QuestPose").setPose(questPose);
            }
            
            // Make more detailed diagnostic data available
            SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent());
            SmartDashboard.putBoolean("Quest Connected", questNav.isConnected());
            SmartDashboard.putBoolean("Quest Pose Seeded", hasQuestInitialized);
            
            // Only show pose if we're connected
            if (questNav.isConnected()) {
                SmartDashboard.putNumberArray("Quest Pose", 
                    new double[] {
                        questNav.getRobotPose().getX(),
                        questNav.getRobotPose().getY(),
                        questNav.getRobotPose().getRotation().getDegrees()
                    });
            }
        }

        // Update trajectory history every few cycles
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
    }

    public Pose2d getPose() {
        //return null;
        return getState().Pose;
    }


    public Pose2d getTagPose(int AprilTagID) {
        return kFieldLayout.getTagPose(AprilTagID).get().toPose2d();
    }


    public Translation2d getTranslationRelative(int apriltagID) { //Repurposed 2024 code
        return 
        getPose().getTranslation()
        .minus(
            kFieldLayout.getTagPose(apriltagID).get().getTranslation().toTranslation2d()
        );
    }

    public Rotation2d getRotationRelative(int apriltagID) { //Taken from 2024 code.
        return 
        getPose().getTranslation()
        .minus(
            getTagPose(apriltagID).getTranslation()
        )
        .unaryMinus()
        .getAngle(); 
    }

    /*
     * Resets the questnav field offset
     * IE: If the quest's 0,0 coordinate is 5,5 on the field coordinate system, then by adding the translation ID 5,5 it will translate questnav's coordinates to feild coordinates
     */
    public void resetQuestPose() {
        questNav.resetPose(this.getPose());
        hasQuestInitialized = true;
    }

    public Command seedQuestPose() {
        return Commands.runOnce(
            () -> {resetQuestPose();}
        );
    }

    public Command disableQuest() {
        return Commands.runOnce(
            () -> {hasQuestInitialized = false;}
        );
    }
    
    public Field2d getField() {
        return this.field;
    }
    
    public QuestNav getQuestNav() {
        return questNav;
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

    // Add method to save calibration values
    public Command saveQuestCalibration() {
        return Commands.runOnce(() -> {
            Translation2d offset = questNav.getCalculatedOffset();
            Preferences.setDouble("QuestOffsetX", offset.getX());
            Preferences.setDouble("QuestOffsetY", offset.getY());
            SmartDashboard.putString("Quest Calibration Status", 
                "Saved offset: [" + offset.getX() + ", " + offset.getY() + "]");
        });
    }

    // Add method to load saved calibration values
    private void loadSavedCalibration() {
        if (Preferences.containsKey("QuestOffsetX") && Preferences.containsKey("QuestOffsetY")) {
            double x = Preferences.getDouble("QuestOffsetX", 0.0);
            double y = Preferences.getDouble("QuestOffsetY", 0.0);
            questNav.applyCalculatedOffset(x, y);
            SmartDashboard.putString("Quest Calibration Status", 
                "Loaded saved offset: [" + x + ", " + y + "]");
        }
    }

    //public double getTranslationRelativeToSpeaker(){
    //    return Math.abs(getPose().getTranslation().getDistance(getSpeakerPose().get().getTranslation().toTranslation2d()));
    //}



    















    


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
}
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

