package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndAffector;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class intakeSource {

    private Drive driveSub;
    private Elevator elevSub;
    private EndAffector endAffectSub;

    // Constructor
    public intakeSource(Drive drive, Elevator elevator, EndAffector endAffector) {
        driveSub = drive;
        elevSub = elevator;
        endAffectSub = endAffector;
    }

    public Command elevatorToPosition(double height) {
        return 
            elevSub.setElevatorGoal(height)
            .andThen(
                elevSub.moveElevator()
                .until(
                    () -> elevSub.elevatorAtGoal()
                )
            );
    }

    public Command intake() {
        return
        new SequentialCommandGroup(
            elevSub.setElevatorGoal(0),
            endAffectSub.loadCoral()
        );     

    }
    
}