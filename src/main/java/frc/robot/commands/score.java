package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class score extends Command{

    private Elevator elevSub;
    private EndAffector endAffectSub;
    private double elevPos;

    // Constructor
    public score(Elevator elevator, EndAffector endAffector, double elevatorPos) {
        elevSub = elevator;
        endAffectSub = endAffector;
        elevPos = elevatorPos;
    }

    public Command scoreCoral () {
        return
        new SequentialCommandGroup(
            elevSub.setElevatorGoal(elevPos),
            Commands.waitSeconds(2.0),
            //endAffectSub.spitCoral(),
            elevSub.setElevatorGoal(0)
        );

    }
    
}