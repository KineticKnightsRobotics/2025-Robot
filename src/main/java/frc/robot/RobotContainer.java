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
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.*;
import frc.robot.commands.Drive.allignApriltag;
import frc.robot.commands.Drive.joystickDrive;
import frc.robot.commands.Drive.scorePosition;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    public Joystick driverController = new Joystick(0);
    public Joystick opPanel = new Joystick(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drive driveSubsystem = TunerConstants.createDrivetrain();
    public final Elevator elevatorSubsystem = new Elevator();
    public final Arm armSubsystem = new Arm();

    // Driver Controller //
    public final Trigger driverA = new Trigger(() -> driverController.getRawButton(1));
    public final Trigger driverB = new Trigger(() -> driverController.getRawButton(2));
    public final Trigger driverX = new Trigger(() -> driverController.getRawButton(3));
    public final Trigger driverY = new Trigger(() -> driverController.getRawButton(4));
    public final Trigger driverStart = new Trigger(() -> driverController.getRawButton(8));

    public final Trigger rightTrigger = new Trigger(() -> driverController.getRawAxis(3) > 0.5);
    public final Trigger leftTrigger = new Trigger(() -> driverController.getRawAxis(2) > 0.5);
    public final Trigger rightBumper = new Trigger(() -> driverController.getRawButton(6));
    public final Trigger leftBumper = new Trigger(() -> driverController.getRawButton(5));
   
    // Operator Panel //
    public final Trigger op1 = new Trigger(() -> opPanel.getRawButton(1));
    public final Trigger op2 = new Trigger(() -> opPanel.getRawButton(2));
    public final Trigger op3 = new Trigger(() -> opPanel.getRawButton(3));
    public final Trigger op4 = new Trigger(() -> opPanel.getRawButton(4));
    public final Trigger op5 = new Trigger(() -> opPanel.getRawButton(5));
    public final Trigger op6 = new Trigger(() -> opPanel.getRawButton(6));
    public final Trigger op7 = new Trigger(() -> opPanel.getRawButton(7));
    public final Trigger op8 = new Trigger(() -> opPanel.getRawButton(8));
    public final Trigger op9 = new Trigger(() -> opPanel.getRawButton(9));
    public final Trigger op10 = new Trigger(() -> opPanel.getRawButton(10));
    public final Trigger op11 = new Trigger(() -> opPanel.getRawButton(11));
    public final Trigger op12 = new Trigger(() -> opPanel.getRawButton(12));
    public final Trigger op13 = new Trigger(() -> opPanel.getRawButton(13));
    public final Trigger op14 = new Trigger(() -> opPanel.getRawButton(14));
    public final Trigger op15 = new Trigger(() -> opPanel.getRawButton(15));
    public final Trigger op16 = new Trigger(() -> opPanel.getRawButton(16));
    public final Trigger op17 = new Trigger(() -> opPanel.getRawButton(17));
    public final Trigger op18 = new Trigger(() -> opPanel.getRawButton(18));
    public final Trigger op19 = new Trigger(() -> opPanel.getRawButton(19));
    public final Trigger op20 = new Trigger(() -> opPanel.getRawButton(20));
    public final Trigger op21 = new Trigger(() -> opPanel.getRawButton(21));

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
        rightBumper.whileTrue(armSubsystem.loadGamePiece(Constants.ArmConstants.intakeSpeed));

        op1.onTrue(scorePosition.score(elevatorSubsystem, armSubsystem, 0.0, 0.0));
        leftBumper.whileTrue(armSubsystem.loadGamePiece(Constants.ArmConstants.intakeSpeed));

        //SmartDashboard.putData("Set Elevator Goal", elevatorSubsystem.setElevatorGoal(10.0));
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
