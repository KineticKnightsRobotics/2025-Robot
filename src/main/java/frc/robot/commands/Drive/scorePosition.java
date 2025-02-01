package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;

public class scorePosition {

    public static Command score(Elevator elevator, Arm arm, double elevatorPos, double armPos) {
        return
        new SequentialCommandGroup(
            elevator.setElevatorGoal(elevatorPos),
            arm.setPivotGoal(armPos)
        );     

    }
    
}