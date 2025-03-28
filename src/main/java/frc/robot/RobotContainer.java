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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.AlgaeAffectorConstants.PivotPositions;
import frc.robot.Constants.ElevatorConstants.Positions;
import frc.robot.commands.multiSubCommands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.Drive.AlignToReefHDC;
//import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drive driveSubsystem = TunerConstants.createDrivetrain();
    public final Elevator elevatorSubsystem = new Elevator();
    public final coralAffector coralSubsystem = new coralAffector();
    public final algaeAffector algaeSubsystem = new algaeAffector();
    public final Climber climberSubsystem = new Climber();
    public final Bling blingSubsystem = new Bling();

    public final multiSubCommands multiSubCommand = new multiSubCommands(driveSubsystem, elevatorSubsystem, coralSubsystem, algaeSubsystem);

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

    public final Trigger lastTwenty = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 20.0;
    });    
    public final Trigger inAuto = new Trigger(() -> DriverStation.isAutonomousEnabled());
    public final Trigger inTeleop = new Trigger(() -> DriverStation.isTeleopEnabled());
    public final Trigger inRegularTeleop = new Trigger(() -> 
        DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 20.0);
    public final Trigger inEndgame = new Trigger(() -> 
        DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 20.0 && DriverStation.getMatchTime() >10);
    public final Trigger lastTenSeconds = new Trigger(() -> 
        DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 10.0 && DriverStation.getMatchTime () > 5);
    public final Trigger lastFiveSeconds = new Trigger(() -> 
        DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 5.0);


    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
        configureLEDTriggers();
        configureNamedCommands();
        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selector",autoSelector);
    }

    public void configureBindings() {

        //driverStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));

        SmartDashboard.putData(multiSubCommand.aimBarge());

        /*
         * DRIVER CONTROLS
         */

        driverA
            .whileTrue(
                algaeSubsystem.spitAlgae()
            );
        //LEFT Reef
        driverY
            //.whileTrue(new allign(driveSubsystem, new Translation2d(Units.inchesToMeters(17.6),0.2),driveSubsystem.getLimelightTarget(),Units.degreesToRadians(180)));
            .whileTrue(
                //multiSubCommand.searchForPeg(-DriveConstants.searchingSpeed,-driverController.getRawAxis(0)*MaxSpeed,-driverController.getRawAxis(4)*AngularRate, search,true)
                multiSubCommand.teleAim(new Translation2d(Units.inchesToMeters(15),0.3),Math.PI,-DriveConstants.searchingSpeed,search)
                    .andThen(blingSubsystem.setLEDAnimation(AnimationTypes.Rainbow))
            )
            .onFalse(
                elevatorSubsystem.elevatorToHeight(Positions.home)
            );
        //RIGHT Reef
        driverX
            //.whileTrue(new allign(driveSubsystem, new Translation2d(Units.inchesToMeters(17.6),-0.2),driveSubsystem.getLimelightTarget(),Units.degreesToRadians(180)));
            .whileTrue(
                //multiSubCommand.searchForPeg(DriveConstants.searchingSpeed,-driverController.getRawAxis(0)*MaxSpeed,-driverController.getRawAxis(4)*AngularRate, search,true)
                multiSubCommand.teleAim(new Translation2d(Units.inchesToMeters(15),-0.3),Math.PI,DriveConstants.searchingSpeed,search)
                    .andThen(blingSubsystem.setLEDAnimation(AnimationTypes.Rainbow))

            )
            .onFalse(
                elevatorSubsystem.elevatorToHeight(Positions.home)
            );

        driverLB
            .whileTrue(
                multiSubCommand.aimBarge()
            )
            .onFalse(
                elevatorSubsystem.elevatorToHeight(Positions.home)
            );

        driverRB
            .whileTrue(
                multiSubCommand.dealgify()
            )
            .onFalse(
                new SequentialCommandGroup(algaeSubsystem.setAlgaePosition(PivotPositions.home),new WaitCommand(1).until(()->algaeSubsystem.atPosition()),elevatorSubsystem.elevatorToHeight(Positions.home))
            );

        driverLT
            .whileTrue(
                algaeSubsystem.intakeAlgae(PivotPositions.groundIntake, PivotPositions.carrying)
            );

        driverRT
            .whileTrue(
                multiSubCommand.intakeCoral()
            )
            .onFalse(elevatorSubsystem.elevatorToHeight(Positions.home).alongWith(algaeSubsystem.setAlgaePosition(PivotPositions.home)));
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

        op2
            .onTrue(
                elevatorSubsystem.setElevatorDealgify(ElevatorConstants.Positions.deAlgifyL3)
            );

        op7
            .onTrue(
                elevatorSubsystem.setElevatorDealgify(ElevatorConstants.Positions.deAlgifyL2)
            );

        /*
         * PROGRAMMER CONTROLS
         */

        //test1.whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        ///test2.whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //test3.whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        //test4.whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        /*
         * TEST PANEL CONTROLS
         */
        /*
        test4
            .whileTrue(
                new AlignToReefHDC(
                    driveSubsystem,
                    new Translation2d(Units.inchesToMeters(15), 0), // 15 inches forward, 0 sideways
                    Math.PI // 180 degrees rotation
                )
            );
            */
    }



    public void configureDefaultCommands() {
            driveSubsystem.setDefaultCommand(
                driveSubsystem.applyRequest(
                    () -> drive
                        .withVelocityX(      ((driverController.getRawAxis(1)*driverController.getRawAxis(1)) * (driverController.getRawAxis(1)>0 ? -1 : 1)) * MaxSpeed) //Square joystick values for finer control with small inputs while still keeping full tilt = full speed
                        .withVelocityY(      ((driverController.getRawAxis(0)*driverController.getRawAxis(0)) * (driverController.getRawAxis(0)>0 ? -1 : 1)) * MaxSpeed)
                        .withRotationalRate( ((driverController.getRawAxis(4)*driverController.getRawAxis(4)) * (driverController.getRawAxis(4)>0 ? -1 : 1)) * AngularRate)
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

        NamedCommands.registerCommand("OptimizedScoreLeft", multiSubCommand.autoAim(-DriveConstants.searchingSpeed, search));
        NamedCommands.registerCommand("OptimizedScoreRight", multiSubCommand.autoAim(DriveConstants.searchingSpeed, search));


        NamedCommands.registerCommand("SigmaScoreLeft",multiSubCommand.sigmaAuto(new Translation2d(Units.inchesToMeters(15),0.3), search));
        NamedCommands.registerCommand("SigmaScoreRight", multiSubCommand.sigmaAuto(new Translation2d(Units.inchesToMeters(15),-0.3), search));

        NamedCommands.registerCommand("ElevatorUp", new SequentialCommandGroup(elevatorSubsystem.setElevatorGoal(ElevatorConstants.Positions.primedHeight),elevatorSubsystem.elevatorToGoal()));
        NamedCommands.registerCommand("ElevatorDown", elevatorSubsystem.elevatorToHeight(Positions.home));

        NamedCommands.registerCommand("ScoreL4", multiSubCommand.scoreCoralAuto(ElevatorConstants.Positions.L4));

        NamedCommands.registerCommand("ScoreL4ProxLeft", multiSubCommand.scoreCoralAutoProx(ElevatorConstants.Positions.L4, DriveConstants.searchingSpeed, search));
        //NamedCommands.registerCommand("ScoreL4ProxLeft", teleopCommand.searchForPeg(-DriveConstants.searchingSpeed, 0.05, 0.0, search,false));
        NamedCommands.registerCommand("ScoreL4ProxRight", multiSubCommand.scoreCoralAutoProx(ElevatorConstants.Positions.L4, -DriveConstants.searchingSpeed, search));
        //NamedCommands.registerCommand("ScoreL4ProxRight", teleopCommand.searchForPeg(DriveConstants.searchingSpeed, 0.05, 0.0, search, false));
    }

    public void configureLEDTriggers() {
        inRegularTeleop.and(dignanHasCoral)
            .onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.GamepieceAquired))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
        inRegularTeleop.and(dignanHasAlgae)
            .onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.GamepieceAquired))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
        inRegularTeleop.and(dignanReefReady)
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.ReadytoScore))
            .onFalse(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
            
        // Auto LED animations
        inAuto
            .onTrue(blingSubsystem.setLEDAnimation(AnimationTypes.AutoDefault));
            
        // Game piece detection during auto - coral only
        inAuto.and(dignanHasCoral)
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.AutoGamePiece));
        
        // This explicitly handles the case when we're in auto but don't have coral
        inAuto.and(dignanHasCoral.negate())
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.AutoDefault));
            
        // Endgame LED animations with color progression based on time and game piece status
        // Last 20-10 seconds
        inEndgame.and(dignanHasCoral.or(dignanHasAlgae))
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameYellowWithGamepiece));
        
        inEndgame.and(dignanHasCoral.or(dignanHasAlgae).negate())
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameYellow));
            
        // Last 10-5 seconds
        lastTenSeconds.and(dignanHasCoral.or(dignanHasAlgae))
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameOrangeWithGamepiece));
        
        lastTenSeconds.and(dignanHasCoral.or(dignanHasAlgae).negate())
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameOrange));
        
        // Last 5 seconds
        lastFiveSeconds.and(dignanHasCoral.or(dignanHasAlgae))
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameRedWithGamepiece));
            
        lastFiveSeconds.and(dignanHasCoral.or(dignanHasAlgae).negate())
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.EndgameRed));
            
        // Ready to score has highest priority
        inEndgame.and(dignanReefReady)
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.ReadytoScore));
        
        // Regular teleop LED animations
        inRegularTeleop.and(dignanHasCoral.or(dignanHasAlgae))
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.GamepieceAquired));
            
        inRegularTeleop.and(dignanReefReady)
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.ReadytoScore));
            
        inRegularTeleop.and(dignanHasCoral.negate().and(dignanHasAlgae.negate()))
            .whileTrue(blingSubsystem.setLEDAnimation(AnimationTypes.Idle));
    }

    public Command getAutonomousCommand() {
        //return new PrintCommand("No Auto LMAO");
        return autoSelector.getSelected();
    }

}
