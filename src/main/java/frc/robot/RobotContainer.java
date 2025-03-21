// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.Bling.AnimationTypes;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Positions;
import frc.robot.commands.multiSubCommands;
//import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    // private final Bling candleSubsystem = new Bling(); <-- This line should be removed

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double AngularRate = Math.PI * 2.5;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    SwerveRequest.RobotCentric search = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Joystick driverController = new Joystick(0);
    public Joystick opPanel = new Joystick(1);
    public Joystick testPanel = new Joystick(2);

    //private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drive driveSubsystem = TunerConstants.createDrivetrain();
    public final Elevator elevatorSubsystem = new Elevator();
    public final coralAffector coralSubsystem = new coralAffector();
    public final algaeAffector algaeSubsystem = new algaeAffector();
    public final Climber climberSubsystem = new Climber();
    public final Bling blingSubsystem = new Bling();

    public final multiSubCommands teleopCommand = new multiSubCommands(driveSubsystem, elevatorSubsystem, coralSubsystem, algaeSubsystem);

    public SendableChooser<Command> autoSelector;

    // Driver Controller //
    public final Trigger driverA = new Trigger(() -> driverController.getRawButton(1));
    public final Trigger driverB = new Trigger(() -> driverController.getRawButton(2));
    public final Trigger driverX = new Trigger(() -> driverController.getRawButton(3));
    public final Trigger driverY = new Trigger(() -> driverController.getRawButton(4));
    public final Trigger driverStart = new Trigger(() -> driverController.getRawButton(8));

    public final Trigger driverRT = new Trigger(() -> driverController.getRawAxis(3) > 0.5);
    public final Trigger driverLT = new Trigger(() -> driverController.getRawAxis(2) > 0.5);
    public final Trigger driverRB = new Trigger(() -> driverController.getRawButton(6));
    public final Trigger driverLB = new Trigger(() -> driverController.getRawButton(5));
   
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
    public final Trigger op10 = new Trigger(() ->opPanel.getRawButton(10));
    public final Trigger op11 = new Trigger(() ->opPanel.getRawButton(11));
    public final Trigger op12 = new Trigger(() ->opPanel.getRawButton(12));
    public final Trigger op13 = new Trigger(() ->opPanel.getRawButton(13));
    public final Trigger op14 = new Trigger(() ->opPanel.getRawButton(14));
    public final Trigger op15 = new Trigger(() ->opPanel.getRawButton(15));
    public final Trigger op16 = new Trigger(() ->opPanel.getRawButton(16));
    public final Trigger op17 = new Trigger(() ->opPanel.getRawButton(17));
    public final Trigger op18 = new Trigger(() ->opPanel.getRawButton(18));
    public final Trigger op19 = new Trigger(() ->opPanel.getRawButton(19));
    public final Trigger op20 = new Trigger(() ->opPanel.getRawButton(20));
    public final Trigger op21 = new Trigger(() ->opPanel.getRawButton(21));
    public final Trigger op22 = new Trigger(() ->opPanel.getRawButton(22));
    public final Trigger op23 = new Trigger(() ->opPanel.getRawButton(23));
    public final Trigger op24 = new Trigger(() ->opPanel.getRawButton(24));

    // Test panel // 
    public final Trigger test1 = new Trigger(() -> testPanel.getRawButton(1));
    public final Trigger test2 = new Trigger(() -> testPanel.getRawButton(2));
    public final Trigger test3 = new Trigger(() -> testPanel.getRawButton(3));
    public final Trigger test4 = new Trigger(() -> testPanel.getRawButton(4));
    public final Trigger test5 = new Trigger(() -> testPanel.getRawButton(5));
    public final Trigger test6 = new Trigger(() -> testPanel.getRawButton(6));
    public final Trigger test7 = new Trigger(() -> testPanel.getRawButton(7));
    public final Trigger test8 = new Trigger(() -> testPanel.getRawButton(8));
    public final Trigger test9 = new Trigger(() -> testPanel.getRawButton(9));
    public final Trigger test10 = new Trigger(() ->testPanel.getRawButton(10));
    //public final EventTrigger canExtend = new EventTrigger("canExtend");
    
    
    
    
    public final Trigger elevatorAtGoal = new Trigger(() -> elevatorSubsystem.elevatorAtGoal());
    public final Trigger dignanHasCoral = new Trigger(() -> coralSubsystem.hasCoral());
    public final Trigger dignanHasAlgae = new Trigger(() -> algaeSubsystem.hasAlgae());

    public final Trigger dignanReefReady = new Trigger(() -> (elevatorSubsystem.elevatorAtGoal() && coralSubsystem.hasCoral() && coralSubsystem.allignedWithPeg()));

    public final Trigger ejectCoral = driverRB.and(dignanHasCoral);
    public final Trigger ejectAlgae = driverRB.and(dignanHasCoral.negate());


    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
        configureNamedCommands();
        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selector",autoSelector);
    }

    public void configureBindings() {

        //driverStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));


        /*
         * DRIVER CONTROLS
         */

        //LEFT Reef
        driverX
            //.whileTrue(new allign(driveSubsystem, new Translation2d(Units.inchesToMeters(17.6),0.2),driveSubsystem.getLimelightTarget(),Units.degreesToRadians(180)));
            .whileTrue(
                //teleopCommand.searchForPeg(-DriveConstants.searchingSpeed,-driverController.getRawAxis(0)*MaxSpeed,-driverController.getRawAxis(4)*AngularRate, search,true)
                teleopCommand.teleAim_Test(new Translation2d(Units.inchesToMeters(17.6),0.3),180,-DriveConstants.searchingSpeed,search)
            );
        //RIGHT Reef
        driverY
            //.whileTrue(new allign(driveSubsystem, new Translation2d(Units.inchesToMeters(17.6),-0.2),driveSubsystem.getLimelightTarget(),Units.degreesToRadians(180)));
            .whileTrue(
                //teleopCommand.searchForPeg(DriveConstants.searchingSpeed,-driverController.getRawAxis(0)*MaxSpeed,-driverController.getRawAxis(4)*AngularRate, search,true)
                teleopCommand.teleAim_Test(new Translation2d(Units.inchesToMeters(17.6),-0.3),180,DriveConstants.searchingSpeed,search)
            );


        driverLB
            .whileTrue(
                elevatorSubsystem.elevatorToGoal()
            )
            .onFalse(
                elevatorSubsystem.elevatorToHeight(Positions.home)
            );
        
        ejectCoral
            .whileTrue(
                coralSubsystem.spitCoral()
            );

        ejectAlgae
            .whileTrue(
                algaeSubsystem.spitAlgae()
            );

        /*
        rightBumper
            .and(dignanHasAlgae)
                .whileTrue(
                    algaeSubsystem.spitAlgae()
                )
            .and(dignanHasCoral)
                .whileTrue(
                    coralSubsystem.spitCoral()
                );
        */

        driverRT
            .whileTrue(
                    new ParallelCommandGroup(
                        coralSubsystem.loadCoral(),
                        elevatorSubsystem.elevatorToHeight(Positions.intake)
                    )
            )
            .onFalse(
                elevatorSubsystem.elevatorToHeight(Positions.home)
            );
        

        /*
         * OPERATOR CONTROLS
         */
        op1
            .onTrue(
                elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.L4)
            );

        op6
            .onTrue(
                elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.L3)
            );

        op11
            .onTrue(
                elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.L2)
            );

        op12
            .onTrue(
                elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.L1)
            );
        

        /*
         * PROGRAMMER CONTROLS
         */
        test1.whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        test2.whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        test3.whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        test4.whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Configure LED bindings based on game piece possession
        dignanHasCoral
            .onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.GamepieceAquired))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
        dignanHasAlgae
            .onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.GamepieceAquired))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
        dignanReefReady
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.ReadytoScore))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));

        //dignanHasCoral.onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.CoralPulse))
        //    .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.SingleFade));
        //dignanHasAlgae.onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.CoralPulse))
        //    .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.SingleFade));
        //last30Seconds.whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.ThirtySeconds));
    }



    public void configureDefaultCommands() {
            driveSubsystem.setDefaultCommand(
                driveSubsystem.applyRequest(
                    () -> drive
                        .withVelocityX(-driverController.getRawAxis(1)*MaxSpeed/**0.2*/)
                        .withVelocityY(-driverController.getRawAxis(0)*MaxSpeed/**0.2*/)
                        .withRotationalRate(-driverController.getRawAxis(4)*AngularRate/**0.2*/)
                    )
            );
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("AquireCoral", 
            new ParallelDeadlineGroup(
                coralSubsystem.loadCoral(),
                elevatorSubsystem.elevatorToHeight(Positions.intake)
            )
        );

        NamedCommands.registerCommand("OptimizedScoreLeft", teleopCommand.autoAim_Test(-DriveConstants.searchingSpeed, search));
        NamedCommands.registerCommand("OptimizedScoreRight", teleopCommand.autoAim_Test(DriveConstants.searchingSpeed, search));

        NamedCommands.registerCommand("ElevatorUp", new SequentialCommandGroup(elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.L4/2),elevatorSubsystem.elevatorToGoal()));
        NamedCommands.registerCommand("ElevatorDown", elevatorSubsystem.elevatorToHeight(Positions.home));
    }

    public Command getAutonomousCommand() {
        //return new PrintCommand("No Auto LMAO");
        return autoSelector.getSelected();
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
