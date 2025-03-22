package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AlgaeAffectorConstants.PivotPositions;
import frc.robot.Constants.ElevatorConstants.Positions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algaeAffector;
import frc.robot.subsystems.coralAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.allign;
import frc.robot.commands.Drive.allignReef;
import edu.wpi.first.math.util.Units;

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
    
    /*
    public Command scoreCoralAuto(double height) {
        return
            new SequentialCommandGroup(
                elevSub.setElevatorGoal(height),
                elevSub.moveElevator()
                    .until(() -> elevSub.elevatorAtGoal()),
                new ParallelRaceGroup(
                    elevSub.moveElevator(),
                    coralSub.spitCoral().withTimeout(1)
                ),
                elevSub.homeElevator()
            );
    }
    */
    public Command scoreCoralAuto(double height) {
        return
            new SequentialCommandGroup(
                //Set elev height
                elevSub.setElevatorGoal(height),
                //Move elevator until its reached its goal. Once its at its goal, we are in the right position to score a coral.
                elevSub.elevatorToGoal().until(()-> elevSub.elevatorAtGoal()),
                //Continue moving elevator until coral has been spat out.
                new ParallelDeadlineGroup(
                    coralSub.spitCoral().withTimeout(1),
                    elevSub.elevatorToGoal()
                ),
                //Send the elevator back to home
                elevSub.elevatorToGoal().until(()-> elevSub.getElevatorPosition() < 10)
            );
    }

    public Command dealgify() { 
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    algaeSub.intakeAlgae(PivotPositions.deAlgifying, PivotPositions.home),
                    elevSub.elevatorToHeight(elevSub.getElevatorAlgaeGoal())
                ),
                elevSub.elevatorToHeight(Positions.home)
            );
    }

    public Command searchForPeg(double searchSpeed, double ySpeed, double rSpeed, SwerveRequest.RobotCentric speedRequest){
        return
            driveSub.applyRequest(
                () -> speedRequest
                    .withVelocityY(searchSpeed)
                    .withVelocityX(ySpeed)
                    .withRotationalRate(rSpeed*0.2)
            ).until(()-> coralSub.allignedWithPeg());
    }



    public Command autoAim_Test(double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            elevSub.setElevatorGoal(Positions.L4),
            new ParallelCommandGroup(
                searchForPeg(searchSpeed, 0.1, 0.0, speedRequest),
                elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal())
            ),
            new ParallelDeadlineGroup(
                coralSub.spitCoral(),
                elevSub.elevatorToGoal()
            )
            
        );
    }

    public Command teleAim_Test(Translation2d desiredDisplacement, double _angleOffset, double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            //elevSub.setElevatorGoal(ElevatorConstants.Positions.primedHeight),
            new ParallelCommandGroup(
                //new allignReef(driveSub, desiredDisplacement, _angleOffset)//, //TODO: Uncomment at drive space for test!
                elevSub.elevatorToHeight(Positions.primedHeight).until(() -> elevSub.elevatorAtGoal())
            ),
            //elevSub.setElevatorGoal(),
            new ParallelCommandGroup(
                searchForPeg(searchSpeed, 0.0, 0.0, speedRequest),
                elevSub.elevatorToGoal()
            ),
            new ParallelCommandGroup(
                coralSub.spitCoral(),
                elevSub.elevatorToGoal()
            )
        );
    }

    public Command aimBarge() {
        return new SequentialCommandGroup(
            //new allign(driveSub, driveSub.getBargePose(), new Translation2d(Units.inchesToMeters(30),0.0), 180), //TODO: Uncomment at drive space for test!
            elevSub.setElevatorGoal(Positions.L4),
            elevSub.elevatorToGoal().until(()->elevSub.elevatorAtGoal()),
            new ParallelCommandGroup(
                algaeSub.setAlgaePosition(PivotPositions.home),
                elevSub.elevatorToGoal()   
            )
        );
    }

    public Command intakeCoral() {
        return new ParallelDeadlineGroup(
            coralSub.loadCoral(),
            elevSub.setElevatorGoal(Positions.intake)
        );
    }



}