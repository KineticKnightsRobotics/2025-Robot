package frc.robot.commands;

import javax.naming.PartialResultException;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AlgaeAffectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Drive.allign;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algaeAffector;
import frc.robot.subsystems.coralAffector;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;

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
                elevSub.moveElevator().until(()-> elevSub.elevatorAtGoal()),
                //Continue moving elevator until coral has been spat out.
                new ParallelDeadlineGroup(
                    coralSub.spitCoral().withTimeout(1),
                    elevSub.moveElevator()
                ),
                //Send the elevator back to home
                elevSub.homeElevator().until(()-> elevSub.getElevatorPosition() < 10)
            );
    }

    public Command deAlgify() { 
        return
            new SequentialCommandGroup(
                algaeSub.setPrimedPosition(AlgaeAffectorConstants.PivotPositions.deAlgifying),
                new ParallelRaceGroup(
                    elevSub.moveElevator(),
                    algaeSub.captureAlgae()
                )
            );
    }

    public Command searchForPeg(double searchSpeed, double ySpeed, double rSpeed, SwerveRequest.RobotCentric speedRequest, boolean flippingLogic){
        return
            driveSub.applyRequest(
                () -> speedRequest
                    .withVelocityY(searchSpeed)
                    .withVelocityX(ySpeed)
                    .withRotationalRate(rSpeed*0.2)
            ).until(()-> coralSub.allignedWithPeg());
    }

    public Command allignToReef_Test(Translation2d desiredDisplacement, double _angleOffset, double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            new allign(driveSub, driveSub.getClosestReefFace(), desiredDisplacement, _angleOffset),
            searchForPeg(-0.2, 0.0, 0.0, speedRequest, false)
        );
    }

    public Command teleAim_Test(Translation2d desiredDisplacement, double _angleOffset, double searchSpeed, double elevHeight, SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            elevSub.setElevatorGoal(ElevatorConstants.Positions.primedHeight),
            new ParallelDeadlineGroup(
                new allign(driveSub, driveSub.getClosestReefFace(), desiredDisplacement, _angleOffset),
                elevSub.moveElevator()
            ),
            elevSub.setElevatorGoal(elevHeight),
            new ParallelCommandGroup(
                searchForPeg(searchSpeed, 0.0, 0.0, speedRequest,false),
                elevSub.moveElevator()
            )
        );
    }

}