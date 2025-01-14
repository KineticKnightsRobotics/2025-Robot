package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;

public class joystickDrive extends Command {
    
    private Drive driveSubsystem;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier zSupplier;

    private final SwerveRequest.FieldCentric speedBuilder = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    public joystickDrive(
        Drive subsystem,
        DoubleSupplier xJoystick,
        DoubleSupplier yJoystick,
        DoubleSupplier zJoystick
    ) {
        addRequirements(subsystem);
        xSupplier = xJoystick;
        ySupplier = yJoystick;
        zSupplier = zJoystick;
        driveSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        addRequirements();
    }

    @Override
    public void execute() {
        driveSubsystem.setControl(
            speedBuilder
                .withVelocityX(xSupplier.getAsDouble()*-maxSpeed*0.2)
                .withVelocityY(ySupplier.getAsDouble()*-maxSpeed*0.2)
                .withRotationalRate(zSupplier.getAsDouble()*-maxAngularRate*0.2)
        );
    }




}
