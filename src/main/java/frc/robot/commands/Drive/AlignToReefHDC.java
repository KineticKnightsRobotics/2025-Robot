package frc.robot.commands.Drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AlignToReefHDC extends Command {
    
    private final Drive mDrive;
    private final Translation2d mTargetOffset;
    private final double mAngleOffset;
    
    private Pose2d mTargetPose;
    private final Timer mTimer = new Timer();
    private final Debouncer mEndTriggerDebouncer = new Debouncer(0.1); // 100ms debounce
    
    // Create error logging publishers
    private final DoublePublisher mXErrorPublisher = NetworkTableInstance.getDefault()
        .getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher mYErrorPublisher = NetworkTableInstance.getDefault()
        .getTable("logging").getDoubleTopic("Y Error").publish();
    
    // Create SwerveRequest for robot-relative speeds
    private final SwerveRequest.ApplyRobotSpeeds speedRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);
    
    /**
     * Creates a command that aligns to the nearest reef using PathPlanner's HolonomicDriveController
     * 
     * @param drive Drive subsystem
     * @param offset Offset from the reef AprilTag in tag-relative coordinates
     * @param angleOffset Angular offset in radians from the tag's orientation
     */
    public AlignToReefHDC(Drive drive, Translation2d offset, double angleOffset) {
        this.mDrive = drive;
        this.mTargetOffset = offset;
        this.mAngleOffset = angleOffset;
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        // Get the closest reef pose
        Pose2d reefPose = mDrive.getClosestReefFace();
        
        // Calculate target pose with offset
        Translation2d offsetRotated = mTargetOffset.rotateBy(reefPose.getRotation());
        mTargetPose = new Pose2d(
            reefPose.getTranslation().plus(offsetRotated),
            reefPose.getRotation().rotateBy(new Rotation2d(mAngleOffset))
        );
        
        mTimer.restart();
        
        SmartDashboard.putNumber("HDC Target X", mTargetPose.getX());
        SmartDashboard.putNumber("HDC Target Y", mTargetPose.getY());
        SmartDashboard.putNumber("HDC Target Angle", mTargetPose.getRotation().getDegrees());
    }
    
    @Override
    public void execute() {
        // Create a trajectory state for the target
        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
        targetState.pose = mTargetPose;
        
        // Calculate speeds using HolonomicDriveController
        ChassisSpeeds speeds = DriveConstants.kHolonomicDriveController.calculateRobotRelativeSpeeds(
            mDrive.getPose(), 
            targetState
        );
        
        // Apply speeds to the drivetrain using CTRE's setControl method
        mDrive.setControl(speedRequest.withSpeeds(speeds));
        
        // Log error information
        double xError = mDrive.getPose().getX() - mTargetPose.getX();
        double yError = mDrive.getPose().getY() - mTargetPose.getY();
        
        mXErrorPublisher.accept(xError);
        mYErrorPublisher.accept(yError);
        
        SmartDashboard.putNumber("HDC X Error", xError);
        SmartDashboard.putNumber("HDC Y Error", yError);
        SmartDashboard.putNumber("HDC Angle Error", 
            mDrive.getPose().getRotation().minus(mTargetPose.getRotation()).getDegrees());
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain using setControl with zero speeds
        mDrive.setControl(speedRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        
        mTimer.stop();
        double elapsed = mTimer.get();
        
        // Calculate final errors
        Pose2d diff = new Pose2d(
            mDrive.getPose().getTranslation().minus(mTargetPose.getTranslation()),
            mDrive.getPose().getRotation().minus(mTargetPose.getRotation())
        );
        
        System.out.println("Reef alignment took: " + elapsed + " seconds, interrupted: " + interrupted
            + "\nPosition error: " + diff.getTranslation().getNorm() * 100 + " cm"
            + "\nRotation error: " + diff.getRotation().getDegrees() + " degrees");
    }
    
    @Override
    public boolean isFinished() {
        // Position error
        double positionError = mDrive.getPose().getTranslation()
            .getDistance(mTargetPose.getTranslation());
            
        // Rotation error in degrees
        double rotationError = Math.abs(mDrive.getPose().getRotation()
            .minus(mTargetPose.getRotation()).getDegrees());
        
        // Check if we've reached the target with acceptable tolerance
        boolean atPosition = positionError < DriveConstants.kHDCPositionTolerance;
        boolean atRotation = rotationError < DriveConstants.kHDCRotationTolerance;
        
        // Also check if proximity sensors detect we're at a reef
        boolean atReef = mDrive.getSensorDig() || mDrive.getSensorNan();
        
        return (atPosition && atRotation) || (atReef && positionError < 0.15);
    }
}
