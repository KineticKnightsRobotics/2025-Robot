package frc.robot.commands;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AlgaeAffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algaeAffector;
import frc.robot.subsystems.coralAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class teleopCommands extends Command{

    private Drive driveSub;
    private Elevator elevSub;
    private coralAffector coralSub;
    private algaeAffector algaeSub;

    // Constructor
    public teleopCommands(Drive drive, Elevator elevator, coralAffector coral, algaeAffector algae) {
        driveSub = drive;
        elevSub = elevator;
        coralSub = coral;
        algaeSub = algae;
    }
    
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
                elevSub.setElevatorGoal(ElevatorConstants.Positions.home),
                elevSub.moveElevator()
                    .until(() -> elevSub.elevatorAtGoal())
            );
    }

    public Command deAlgify(double height) { 
        return
            new SequentialCommandGroup(
                elevSub.setElevatorGoal(height),
                elevSub.moveElevator()
                    .until(()-> elevSub.elevatorAtGoal()),
                new ParallelRaceGroup(
                    elevSub.moveElevator(),
                    algaeSub.captureAlgae(AlgaeAffectorConstants.PivotPositions.deAlgifying)
                ) ,
                elevSub.setElevatorGoal(ElevatorConstants.Positions.home)
            );
    }
}