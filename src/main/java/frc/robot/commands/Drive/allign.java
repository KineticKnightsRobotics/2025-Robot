package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.AlignmentController.RotationController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeXController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeYController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;

public class allign extends Command {
    
    private Drive driveSubsystem;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private Translation2d displacement;
    private int targetApriltag;
    private Pose2d fieldCoordinate;
    private double angleOffset;

    private PIDController xController = new PIDController(StrafeXController.P,StrafeXController.I,StrafeXController.D);
    private PIDController yController = new PIDController(StrafeYController.P,StrafeYController.I,StrafeYController.D);
    private PIDController rController = new PIDController(RotationController.P,RotationController.I,RotationController.D);

    private final SwerveRequest.FieldCentric speedBuilder = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    public allign(
        Drive kSubsystem,
        Translation2d desiredDisplacement,
        int apriltagID,
        double _angleOffset
    ) {
        addRequirements(kSubsystem);
        driveSubsystem = kSubsystem;
        displacement = desiredDisplacement;
        targetApriltag = apriltagID;
        angleOffset = _angleOffset;

    }

    @Override
    public void initialize() {
        Pose2d tagPose = driveSubsystem.getTagPose(targetApriltag);
        Translation2d displacementRotated = displacement.rotateBy(tagPose.getRotation());
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



        double[] AllignmentOutput = {outputX, outputY, outputR};
        SmartDashboard.putNumberArray("Allignment PID Outputs", AllignmentOutput);

        driveSubsystem.setControl(
            speedBuilder
                .withVelocityX(outputX)
                .withVelocityY(outputY)
                .withRotationalRate(outputR)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
