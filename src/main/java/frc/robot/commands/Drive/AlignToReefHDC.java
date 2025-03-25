package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AlignToReefHDC extends Command {
    
    private final Drive driveSubsystem;
    private final Pose2d goalPose;
    private final PPHolonomicDriveController driveController = DriveConstants.kHolonomicDriveController;
private final SwerveRequest.ApplyRobotSpeeds speedRequest = new SwerveRequest.ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity);  // Add this line to match Drive.java
    private final Timer timer = new Timer();
    private final Debouncer endTriggerDebouncer = new Debouncer(0.1); // 100ms debounce
    
    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();
    
    /**
     * Creates a command to align to a reef with a holonomic drive controller.
     * 
     * @param drive The drive subsystem
     * @param displacement The displacement from the nearest reef face
     * @param angle The angle offset from the reef face (in radians)
     */
    public AlignToReefHDC(Drive drive, Translation2d displacement, double angle) {
        driveSubsystem = drive;
        
        // Calculate goal pose based on reef face and desired position
        Pose2d reefFace = drive.getClosestReefFace();
        Translation2d displacementRotated = displacement.rotateBy(reefFace.getRotation());
        goalPose = new Pose2d(
            reefFace.getTranslation().plus(displacementRotated),
            reefFace.getRotation().rotateBy(new Rotation2d(angle))
        );
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        timer.restart();
    }
    
    @Override
public void execute() {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = goalPose;
    
    ChassisSpeeds speeds = driveController.calculateRobotRelativeSpeeds(
        driveSubsystem.getPose(), goalState
    );
    
    driveSubsystem.setControl(speedRequest.withSpeeds(speeds));
    
    xErrLogger.accept(driveSubsystem.getPose().getX() - goalPose.getX());
    yErrLogger.accept(driveSubsystem.getPose().getY() - goalPose.getY());
}
    
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        
        // Stop the robot
        driveSubsystem.setControl(speedRequest.withSpeeds(new ChassisSpeeds()));
        
        // Log results
        Pose2d diff = driveSubsystem.getPose().relativeTo(goalPose);
        
        System.out.println("Reef alignment took: " + timer.get() + " seconds, interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getDegrees() + " deg"
            + "\nVelocity: " + driveSubsystem.getState().Speeds.vxMetersPerSecond + ", " + driveSubsystem.getState().Speeds.vyMetersPerSecond + " m/s"
        );
    }
    
    @Override
    public boolean isFinished() {
        Pose2d diff = driveSubsystem.getPose().relativeTo(goalPose);
        
        // Check if rotation is within tolerance
        boolean rotationAligned = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRadians(), 
            Math.toRadians(3.0), // 3 degree tolerance
            0.0, 
            Math.PI
        );
        
        // Check if position is within tolerance
        boolean positionAligned = diff.getTranslation().getNorm() < 0.05; // 5cm tolerance
        
        // Check if robot is mostly stopped
        double speed = Math.hypot(
            driveSubsystem.getState().Speeds.vxMetersPerSecond,
            driveSubsystem.getState().Speeds.vyMetersPerSecond
        );
        boolean speedSettled = speed < 0.1; // 10cm/s tolerance
        
        // Also check the proximity sensors to see if we've found a branch
        boolean proximityDetected = driveSubsystem.getSensorDig() || driveSubsystem.getSensorNan();
        
        // We're done if all conditions are met
        return endTriggerDebouncer.calculate(
            rotationAligned && positionAligned && speedSettled && proximityDetected
        );
    }
}
