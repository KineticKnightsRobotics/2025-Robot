package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.PIDConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.AlignmentController.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;

public class allignApriltag extends Command {
    
    private Drive driveSubsystem;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private Translation2d targetCoordinate;
    private int targetApriltag;



    private PIDController xController = new PIDController(StrafeXController.P,StrafeXController.I,StrafeXController.D);

    private PIDController yController = new PIDController(StrafeYController.P,StrafeYController.I,StrafeYController.D);

    private PIDController rController = new PIDController(RotationController.P,RotationController.I,RotationController.D);



    private final SwerveRequest.FieldCentric speedBuilder = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    public allignApriltag(
        Drive kSubsystem,
        Translation2d desiredDisplacement,
        int apriltagID
    ) {
        addRequirements(kSubsystem);
        driveSubsystem = kSubsystem;
        targetCoordinate = desiredDisplacement;
        targetApriltag = apriltagID;
    }

    @Override
    public void initialize() {
        targetCoordinate = targetCoordinate
            .rotateBy(
                driveSubsystem.getRotationRelative(targetApriltag)
            ).plus(
                driveSubsystem.getTagPose(targetApriltag)
            );
    }

    @Override
    public void execute() {

        Pose2d goalPose = new Pose2d(targetCoordinate, new Rotation2d(0.0));

        double outputX = xController.calculate(
                driveSubsystem.getTranslationRelative(targetApriltag).getX(),
                targetCoordinate.getX()
            );

        double outputY = yController.calculate(
                driveSubsystem.getTranslationRelative(targetApriltag).getY(),
                targetCoordinate.getY()
            );

        double outputR = rController.calculate(
                driveSubsystem.getPose().getRotation().getDegrees(),
                driveSubsystem.getRotationRelative(targetApriltag).getDegrees()
            );


        SmartDashboard.putNumber("apriltag X output", outputX);

        driveSubsystem.setControl(
            speedBuilder
                .withVelocityX(outputX)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}