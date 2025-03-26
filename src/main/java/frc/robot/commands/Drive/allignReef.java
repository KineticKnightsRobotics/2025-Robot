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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants.VisionConstants.AlignmentController.RotationController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeXController;
import frc.robot.Constants.VisionConstants.AlignmentController.StrafeYController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;

public class allignReef extends Command {
    
    private Drive driveSubsystem;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private Translation2d poseOffset;
    private Pose2d fieldCoordinate;
    private double angleOffset;
    private Pose2d tagPose;

    private PIDController xController = new PIDController(StrafeXController.P,StrafeXController.I,StrafeXController.D);
    private PIDController yController = new PIDController(StrafeYController.P,StrafeYController.I,StrafeYController.D);
    private PIDController rController = new PIDController(RotationController.P,RotationController.I,RotationController.D);

    private double outputX;
    private double outputY;
    private double outputR;

    private double error;
    
    // Debouncer for isFinished condition - 0.1 seconds
    private final Debouncer finishedDebouncer = new Debouncer(0.1, DebounceType.kRising);

    private final SwerveRequest.FieldCentric speedBuilder = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    public allignReef(
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
        tagPose = driveSubsystem.getClosestReefFace();
        Translation2d displacementRotated = poseOffset.rotateBy(tagPose.getRotation());
        fieldCoordinate = new Pose2d(tagPose.getTranslation().plus(displacementRotated), tagPose.getRotation().rotateBy(new Rotation2d(angleOffset)));
    }

    @Override
    public void execute() {
        outputX = xController.calculate(
            driveSubsystem.getPose().getX(), fieldCoordinate.getX()
        );
        outputY = yController.calculate(
            driveSubsystem.getPose().getY(), fieldCoordinate.getY()
        );
        outputR = rController.calculate(
            driveSubsystem.getPose().getRotation().getDegrees(), fieldCoordinate.getRotation().getDegrees()
        );

        //double[] AllignmentOutput = {outputX, outputY, outputR};
        //SmartDashboard.putNumberArray("Allignment PID Outputs", AllignmentOutput);

        // Calculate error
        error = driveSubsystem.getPose().getTranslation().getDistance(fieldCoordinate.getTranslation());

        // Output values to SmartDashboard for graphing
        SmartDashboard.putNumber("Alignment/Error", error);
        SmartDashboard.putNumber("Alignment/SensorDig", driveSubsystem.getSensorDig() ? 1.0 : 0.0);
        SmartDashboard.putNumber("Alignment/SensorNan", driveSubsystem.getSensorNan() ? 1.0 : 0.0);
        SmartDashboard.putNumber("Alignment/OutputX", outputX);
        SmartDashboard.putNumber("Alignment/OutputY", outputY);
        SmartDashboard.putNumber("Alignment/OutputR", outputR);

        driveSubsystem.setControl(
            speedBuilder
                .withVelocityX(outputX)
                .withVelocityY(outputY)
                .withRotationalRate(outputR)
        );
    }

    @Override
    public boolean isFinished() {
        return (driveSubsystem.getSensorDig() || driveSubsystem.getSensorNan()) && error < 0.10;
        
        //boolean sensorsActivated = driveSubsystem.getSensorDig() || driveSubsystem.getSensorNan();
        //boolean withinErrorTolerance = error < 0.10;
        //return finishedDebouncer.calculate(sensorsActivated && withinErrorTolerance);
    }
}
