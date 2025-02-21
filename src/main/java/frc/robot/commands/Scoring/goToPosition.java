package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

public class goToPosition {





    public Command elevatorToPosition(Elevator elv, double height) {
        return Commands
            .run(
                () -> {
                    elv.setElevatorGoal(height);
                },
                elv    
            );
    }
}
