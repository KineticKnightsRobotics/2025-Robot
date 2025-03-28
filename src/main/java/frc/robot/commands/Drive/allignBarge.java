package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.AlignmentController.RotationController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeXController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeYController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.util.ReefSelector;

public class allignBarge extends Command {
    
    private Drive driveSubsystem;

    //private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    ///private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private Translation2d poseOffset;
    private Pose2d fieldCoordinate;
    private double angleOffset;
    private Pose2d tagPose;

    private double error;

    private PIDController xController = new PIDController(StrafeXController.P,StrafeXController.I,StrafeXController.D);
    private PIDController yController = new PIDController(StrafeYController.P,StrafeYController.I,StrafeYController.D);
    private PIDController rController = new PIDController(RotationController.P,RotationController.I,RotationController.D);

    private final SwerveRequest.FieldCentric speedBuilder = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    public allignBarge(
        Drive kSubsystem,
        Translation2d displacement,
        double _angleOffset
    ) {
        addRequirements(kSubsystem);
        driveSubsystem = kSubsystem;
        poseOffset = displacement;
        angleOffset = _angleOffset;

        rController.enableContinuousInput(-179, 180);
    }

    @Override
    public void initialize() {
        tagPose = driveSubsystem.getBargePose();
        Translation2d displacementRotated = poseOffset.rotateBy(tagPose.getRotation());
        fieldCoordinate = new Pose2d(tagPose.getTranslation().plus(displacementRotated), tagPose.getRotation().rotateBy(new Rotation2d(angleOffset)));
    }

    @Override
    public void execute() {

        double outputX = xController.calculate(
            driveSubsystem.getPose().getX(), fieldCoordinate.getX()
        );
        double outputY = yController.calculate(
            driveSubsystem.getPose().getY(), fieldCoordinate.getY()
        );
        double outputR = rController.calculate(
            driveSubsystem.getPose().getRotation().getDegrees(), fieldCoordinate.getRotation().getDegrees()
        );

        if (driveSubsystem.reefSelector.redAlliance) {
            outputX *= -1; outputY *= -1; //outputR *= -1;
        }

        //double[] AllignmentOutput = {outputX, outputY, outputR};
        //SmartDashboard.putNumberArray("Allignment PID Outputs", AllignmentOutput);

        error = driveSubsystem.getPose().getTranslation().getDistance(fieldCoordinate.getTranslation());

        driveSubsystem.setControl(
            speedBuilder
                .withVelocityX(outputX)
                .withVelocityY(outputY)
                .withRotationalRate(outputR)
        );
    }

    @Override
    public boolean isFinished() {
        Transform2d TargetError = fieldCoordinate.minus(driveSubsystem.getPose());
        return driveSubsystem.getSensorNan() && error < 0.10;
    }
}
