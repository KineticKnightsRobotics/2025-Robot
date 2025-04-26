package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AlgaeAffectorConstants.PivotPositions;
import frc.robot.Constants.ElevatorConstants.Positions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algaeAffector;
import frc.robot.subsystems.coralAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;

public class multiSubCommands extends Command{

    private Drive driveSub;
    private Elevator elevSub;
    private coralAffector coralSub;
    private algaeAffector algaeSub;

    // Constructor
    public multiSubCommands(Drive drive, Elevator elevator, coralAffector coral, algaeAffector algae) {
        driveSub = drive;
        elevSub = elevator;
        coralSub = coral;
        algaeSub = algae;
    }

    public Command intakeCoral() {
        return new SequentialCommandGroup(
            algaeSub.setAlgaePosition(PivotPositions.coralIntaking),
            new WaitCommand(0.1),
            new ParallelDeadlineGroup(
                coralSub.loadCoral()
                //elevSub.elevatorToHeight(Positions.intake)
            ),
            algaeSub.setAlgaePosition(PivotPositions.home)
           // elevSub.elevatorToHeight(Positions.home)
        );
    }

    public Command dealgify() { 
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    algaeSub.intakeAlgae(PivotPositions.deAlgifying, PivotPositions.home),
                    elevSub.elevatorToAlgae()
                ),
                algaeSub.setAlgaePosition(PivotPositions.home),
                new WaitCommand(0.5).until(()->algaeSub.atPosition()),
                elevSub.elevatorToHeight(Positions.home)
            );
    }

    public Command dealgifyAuto(double dealgifyLevel) { 
        return
            new SequentialCommandGroup(
                elevSub.setElevatorDealgify(dealgifyLevel),
                elevSub.elevatorToAlgae().until(()-> dealgifyLevel - elevSub.getElevatorPosition() > 1.5),
                new ParallelDeadlineGroup(
                    algaeSub.intakeAlgae(PivotPositions.deAlgifying, PivotPositions.home),
                    elevSub.elevatorToAlgae()
                ),
                algaeSub.setAlgaePosition(PivotPositions.home),
                new WaitCommand(0.5).until(()->algaeSub.atPosition()),
                elevSub.elevatorToHeight(Positions.home)
            );
    }

    private Debouncer debouncer = new Debouncer(0.04, DebounceType.kBoth);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public Command searchForPegTele(double searchSpeed, double ySpeed, double rSpeed, SwerveRequest.RobotCentric speedRequest){
        return
            driveSub.applyRequest(
                () -> speedRequest
                    .withVelocityY(searchSpeed*MaxSpeed)
                    .withVelocityX(0.0)
                    .withRotationalRate(0.0)
            ).until(()-> debouncer.calculate(coralSub.allignedWithPeg()))
            .andThen(new WaitCommand(/*0.05*/0.001))
            .andThen(
                ()-> speedRequest
                    .withVelocityY(searchSpeed * -1)
                    .withVelocityX(0.0)
                    .withRotationalRate(0.0)
            ).until(()-> debouncer.calculate(coralSub.allignedWithPeg()))
            .andThen(
                Commands.runOnce(() -> driveSub.setControl(speedRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)))
            );
    }
    public Command searchForPegAuto(double direction, SwerveRequest.RobotCentric speedRequest) {
        return 
            new SequentialCommandGroup(
                driveSub.applyRequest(
                    () -> speedRequest
                        .withVelocityY(DriveConstants.searchingSpeedAutoSeek * direction)
                        .withVelocityX(0.0)
                        .withRotationalRate(0.0)
                ).until(()-> coralSub.allignedWithPeg()),
                driveSub.applyRequest(
                    () -> speedRequest
                        .withVelocityY(DriveConstants.searchingSpeedAutoSeek * direction)
                        .withVelocityX(0.0)
                        .withRotationalRate(0.0)
                ).until(()-> !coralSub.allignedWithPeg()),
                driveSub.applyRequest(
                    () -> speedRequest
                        .withVelocityY(DriveConstants.searchingSpeedAutoFine * direction * -1)
                        .withVelocityX(0.0)
                        .withRotationalRate(0.0)
                ).until(()-> coralSub.allignedWithPeg()),
                driveSub.applyRequest(
                    () -> speedRequest
                        .withVelocityY(0.0)
                        .withVelocityX(0.0)
                        .withRotationalRate(0.0)
                )
            );
    }

    public Command scoreCoralAutoProx(double height, double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return
            new SequentialCommandGroup(
                //Set elev height
                elevSub.setElevatorGoal(height),
                //Move elevator until its reached its goal. Once its at its goal, we are in the right position to score a coral.
                elevSub.elevatorToGoal().until(()-> elevSub.elevatorAtGoal()),
                new ParallelDeadlineGroup(
                    searchForPegTele(searchSpeed,0.0,0.0,speedRequest),
                    elevSub.elevatorToGoal()
                ),
                //Continue moving elevator until coral has been spat out.
                new ParallelDeadlineGroup(
                    coralSub.spitCoral().withTimeout(1),
                    elevSub.elevatorToGoal()
                )
                //Send the elevator back to home
            );
    }

    /*
    public Command sigmaAuto(Translation2d desiredDisplacement, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            //elevSub.setElevatorGoal(ElevatorConstants.Positions.primedHeight),
            new ParallelCommandGroup(
                new allignReef(driveSub, desiredDisplacement, Math.PI),
                elevSub.elevatorToHeight(Positions.primedHeight).until(() -> elevSub.getElevatorPosition() > Positions.primedHeight-1.5)
            ),
            elevSub.setElevatorGoal(Positions.L4),
            //elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal()),
            new ParallelDeadlineGroup(
                new searchForBranch(driveSub, coralSub),
                elevSub.elevatorToGoal()//.until(()->(elevSub.getElevatorGoal() > 56))
            ),
            elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal()),
            new ParallelDeadlineGroup(
                coralSub.spitCoral(),
                elevSub.elevatorToGoal()
            )
        );
    }

    public Command teleAim(Translation2d desiredDisplacement, double _angleOffset, double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            //elevSub.setElevatorGoal(ElevatorConstants.Positions.primedHeight),
            new ParallelCommandGroup(
                new allignReef(driveSub, desiredDisplacement, _angleOffset),
                elevSub.elevatorToHeight(Positions.primedHeight).until(() -> elevSub.getElevatorPosition() > Positions.primedHeight-1.5)
            ),
            elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal()),
            new ParallelDeadlineGroup(
                new searchForBranch(driveSub, coralSub),
                elevSub.elevatorToGoal()//.until(()->(elevSub.getElevatorGoal() > 56))
            ),
            new WaitCommand(0.15),
            new ParallelCommandGroup(
                coralSub.spitCoral(),
                elevSub.elevatorToGoal()
            )
        );
    }
    

    public Command aimBarge() {
        return new SequentialCommandGroup(
            //new allignBarge(driveSub, new Translation2d(Units.inchesToMeters(30),0.0), Math.PI),
            elevSub.setElevatorGoal(Positions.L4),
            elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal()),
            new ParallelCommandGroup(
                algaeSub.setAlgaePosition(PivotPositions.home),
                elevSub.elevatorToGoal()   
            )
        );
    }
    */
}