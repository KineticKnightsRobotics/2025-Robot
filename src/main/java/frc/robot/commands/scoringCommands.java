package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class scoringCommands extends Command{

    private Drive driveSub;
    private Elevator elevSub;
    private EndAffector endAffectSub;

    // Constructor
    public scoringCommands(Drive drive, Elevator elevator, EndAffector endAffector) {
        driveSub = drive;
        elevSub = elevator;
        endAffectSub = endAffector;
    }

    public Command scoreCoral (double elevPos) {
        return
        new SequentialCommandGroup(
            elevSub.setElevatorGoal(elevPos),
            Commands.waitSeconds(2.0),
            //endAffectSub.spitCoral(),
            elevSub.setElevatorGoal(0)
        );

    }

    public Command elevatorToPosition(Elevator elv, double height) {
        return 
            elv.setElevatorGoal(height)
            .andThen(
                elv.moveElevator()
                .until(
                    () -> elv.elevatorAtGoal()
                )
            );
    }


    /*
    /**
     * 
     * @param elv Elevator Subsystem
     * @param drv Drive Subsystem
     * @param aff Affector Subsystem
     * @param allignmentVector Vector for allignment
     * @param elevatorHeight height to go to for elevator
     * @return
     
    public Command autoScore(
                Translation2d allignmentVector,
                double elevatorHeight
            ) {
        return 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new allign(drv, null, 0, elevatorHeight),
                    elevatorToPosition(elv, elevatorHeight)
                ),
                aff.spitCoral()
            );
    }
    */


    
}