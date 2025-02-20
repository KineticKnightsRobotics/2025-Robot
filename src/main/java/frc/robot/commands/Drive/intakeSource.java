package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class intakeSource {

    private Elevator elevSub;
    private EndAffector endAffectSub;

    // Constructor
    public intakeSource(Elevator elevator, EndAffector endAffector) {
        elevSub = elevator;
        endAffectSub = endAffector;

    }

    public Command intake() {
        return
        new SequentialCommandGroup(
            elevSub.setElevatorGoal(0),
            endAffectSub.loadCoral()
        );     

    }
    
}