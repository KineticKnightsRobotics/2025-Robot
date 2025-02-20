// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.commands.*;
import frc.robot.commands.Drive.allign;
import frc.robot.commands.Drive.elevatorSysIDCommand;
import frc.robot.commands.Drive.joystickDrive;
import frc.robot.commands.Drive.score;
import frc.robot.commands.Drive.intakeSource;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double AngularRate = Math.PI * 1.5;
    private SwerveRequest joystickDrive;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Joystick driverController = new Joystick(0);
    public Joystick opPanel = new Joystick(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drive driveSubsystem = TunerConstants.createDrivetrain();
    public final Elevator elevatorSubsystem = new Elevator();
    public final Arm armSubsystem = new Arm();
    public final EndAffector endAffectorSubsytem = new EndAffector();
    public final Climber climberSubsystem = new Climber();

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
    public final Trigger op22 = new Trigger(() -> opPanel.getRawButton(22));
    public final Trigger op23 = new Trigger(() -> opPanel.getRawButton(23));


    public final Trigger elevatorAtGoal = new Trigger(() -> elevatorSubsystem.elevatorAtGoal());


    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();

        // NamedCommands
        NamedCommands.registerCommand("scoreStage1", new score(elevatorSubsystem, endAffectorSubsytem, ElevatorConstants.stage1).scoreCoral());
        NamedCommands.registerCommand("scoreStage2", new score(elevatorSubsystem, endAffectorSubsytem, ElevatorConstants.stage2).scoreCoral());
        NamedCommands.registerCommand("intakeSource", new intakeSource(elevatorSubsystem, endAffectorSubsytem).intake());
    }

    public void configureBindings() {

        driverStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));


        op3
            .whileTrue(
                elevatorSubsystem.setElevatorGoal(ElevatorConstants.ChassisElevationOffset+1)
            );


        op6
            .onTrue(
                elevatorSubsystem.setElevatorGoal(38)
            );
        
        op17
            .onTrue(elevatorSubsystem.setElevatorGoal(ElevatorConstants.ChassisElevationOffset+1));
        op18
            .onTrue(endAffectorSubsytem.spitCoral());


        op9.onTrue(armSubsystem.setPivotGoal(90));

        /*
        //arm
        op1.onTrue(armSubsystem.setPivotGoal(1.0));
        op2.onTrue(armSubsystem.setPivotGoal(15.0));
        op3.onTrue(armSubsystem.setPivotGoal(50.0));
        op6.onTrue(armSubsystem.setPivotGoal(47.67));
        op7.onTrue(armSubsystem.setPivotGoal(70));
        op8.onTrue(armSubsystem.setPivotGoal(90.0));
        //elevator
        op11.whileTrue(elevatorSubsystem.setElevatorGoal(ElevatorConstants.ChassisElevationOffset+1));
        op12.whileTrue(elevatorSubsystem.setElevatorGoal(40));
        op13.whileTrue(elevatorSubsystem.setElevatorGoal(15));
        op15.whileTrue(elevatorSubsystem.setElevatorGoal(56));
        op14.whileTrue(elevatorSubsystem.setElevatorGoal(30));
        //end affector
        op17.whileTrue(endAffectorSubsytem.loadCoral());
        op18.whileTrue(endAffectorSubsytem.spitCoral());
        op19.whileTrue(endAffectorSubsytem.loadAlgae());
        op20.whileTrue(endAffectorSubsytem.spitAlgae());
        */

        op21.whileTrue(new allign(driveSubsystem, new Translation2d(1,0), 1));

        //climber
        op4.whileTrue(climberSubsystem.setClimberSpeed(1.0));
        op5.whileTrue(climberSubsystem.setClimberSpeed(-1.0));





        //op22.whileTrue(new elevatorSysIDCommand(elevatorSubsystem, ()->0.02));
        //op23.whileTrue(new elevatorSysIDCommand(elevatorSubsystem, ()->-0.02));

        //not working yet
        //op4.whileTrue(scorePosition.score(elevatorSubsystem, armSubsystem, 55, 15));
        //op5.whileTrue(scorePosition.score(elevatorSubsystem, armSubsystem, ElevatorConstants.ChassisElevationOffset, 90));

        //SmartDashboard.putData("Set Elevator Goal", elevatorSubsystem.setElevatorGoal(10.0));
    }

    public void configureDefaultCommands() {
        /*
        driveSubsystem.setDefaultCommand(
            new joystickDrive(
                driveSubsystem,
                () -> driverController.getRawAxis(0),
                () -> driverController.getRawAxis(1),
                () -> driverController.getRawAxis(4)
            )
        );
        */

        driveSubsystem.setDefaultCommand(
            driveSubsystem.applyRequest(
                () -> drive
                    .withVelocityX(-driverController.getRawAxis(1)*MaxSpeed*0.2)
                    .withVelocityY(-driverController.getRawAxis(0)*MaxSpeed*0.2)
                    .withRotationalRate(-driverController.getRawAxis(4)*MaxSpeed*0.2)
                )
        );

        elevatorSubsystem.setDefaultCommand(
            elevatorSubsystem.moveElevator(() -> (armSubsystem.getPivotEncoderPosition() < 85))
        );
        armSubsystem.setDefaultCommand(
            armSubsystem.pivotArm().withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("scoreTest");
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
