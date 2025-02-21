package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.allign;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndAffector;

public class goToPosition {

    public Command elevatorToPosition(Elevator elv, double height) {
        return Commands
            .runOnce(
                () -> {
                    elv.setElevatorGoal(height);
                },
                elv
            )
            .andThen(
                Commands.run(
                    () -> {
                        elv.moveElevator();
                    },
                    elv
                )
                .until(
                    () -> elv.elevatorAtGoal()
                )
            );
    }


    /**
     * 
     * @param elv Elevator Subsystem
     * @param drv Drive Subsystem
     * @param aff Affector Subsystem
     * @param allignmentVector Vector for allignment
     * @param elevatorHeight height to go to for elevator
     * @return
     */
    public Command autoScore(
                Elevator elv,
                Drive drv,
                EndAffector aff,
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
}


