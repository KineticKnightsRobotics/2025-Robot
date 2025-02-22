package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ElevatorConstants;
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

    public Command score(double height) {
        return
            new SequentialCommandGroup(
                elevSub.setElevatorGoal(height).andThen(elevSub.moveElevator().until(() -> elevSub.elevatorAtGoal())),
                new ParallelRaceGroup(elevSub.moveElevator(), endAffectSub.loadCoral().withTimeout(2)),
                elevSub.setElevatorGoal(ElevatorConstants.ChassisElevationOffset+1),
                elevSub.moveElevator().until(() -> elevSub.elevatorAtGoal())
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