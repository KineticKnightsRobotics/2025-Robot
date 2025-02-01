// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
//import frc.robot.commands.*;
import frc.robot.commands.Drive.allignApriltag;
import frc.robot.commands.Drive.joystickDrive;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    public Joystick driverController = new Joystick(0);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drive driveSubsystem = TunerConstants.createDrivetrain();
    public final Elevator elevatorSubsystem = new Elevator();
    public final Arm armSubsystem = new Arm();

    public final Trigger driverA = new Trigger(() -> driverController.getRawButton(1));
    public final Trigger driverB = new Trigger(() -> driverController.getRawButton(2));
    public final Trigger driverX = new Trigger(() -> driverController.getRawButton(3));
    public final Trigger driverY = new Trigger(() -> driverController.getRawButton(4));
    public final Trigger driverStart = new Trigger(() -> driverController.getRawButton(8));

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    public void configureBindings() {

    
        //driverA.onTrue(elevatorSubsystem.setElevatorGoal(15));
        //driverA.whileTrue(new allignApriltag(driveSubsystem, new Translation2d(0, -1), 1));
        //driverB.onTrue(elevatorSubsystem.setElevatorGoal(0));
        //driverX.onTrue(elevatorSubsystem.setElevatorGoal(50));
        //driverY.onTrue(elevatorSubsystem.setElevatorGoal(30));
        driverStart.onTrue(elevatorSubsystem.zeroElevatorPosition());
        driverA.onTrue(elevatorSubsystem.setElevatorGoal(0));
        driverB.onTrue(armSubsystem.setPivotGoal(0));
        driverX.onTrue(armSubsystem.setPivotGoal(50));
        driverY.onTrue(armSubsystem.setPivotGoal(Constants.ArmConstants.idlePosition));



        SmartDashboard.putData("Set Elevator Goal", elevatorSubsystem.setElevatorGoal(10.0));
    }

    public void configureDefaultCommands() {
        driveSubsystem.setDefaultCommand(
            new joystickDrive(
                driveSubsystem,
                () -> driverController.getRawAxis(0),
                () -> driverController.getRawAxis(1),
                () -> driverController.getRawAxis(4)
            )
        );
        elevatorSubsystem.setDefaultCommand(
            elevatorSubsystem.moveElevator()
        );
        armSubsystem.setDefaultCommand(
            armSubsystem.pivotArm()
        );

        

    }

    public Command getAutonomousCommand() {
        return Commands.print("No Auto LMAO");
    }
}


    /*
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
    */
