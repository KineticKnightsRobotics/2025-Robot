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

    public Command scoreCoralAutoProx(double height, double searchSpeed, SwerveRequest.RobotCentric speedRequest) {
        return
            new SequentialCommandGroup(
                //Set elev height
                elevSub.setElevatorGoal(height),
                //Move elevator until its reached its goal. Once its at its goal, we are in the right position to score a coral.
                elevSub.moveElevator().until(()-> elevSub.elevatorAtGoal()),
                searchForPeg(searchSpeed,0.0,0.0,speedRequest,false),
                //Continue moving elevator until coral has been spat out.
                new ParallelDeadlineGroup(
                    coralSub.spitCoral().withTimeout(1),
                    elevSub.moveElevator()
                ),
                //Send the elevator back to home
                elevSub.homeElevator().until(()-> elevSub.getElevatorPosition() < 10)
            );
    }

    public Command scoreCoralAuto_Optimized(double height, double searchSpeed , SwerveRequest.RobotCentric speedRequest) {
        return new SequentialCommandGroup(
            // Set the first goal to a height where the prox sensor works correctly
            elevSub.setElevatorGoal(11.5),

            // Will not finish until the elevator is at least at the goal (or above)
            elevSub.moveElevator().until(() -> (elevSub.getElevatorPosition() > 10)),

            // Seek and finish extending
            new ParallelCommandGroup(
                // Move the elevator until it has reached scoring position
                elevSub.moveElevator().until(() -> elevSub.elevatorAtHeight(height)),

                // Line up with peg and maintain elevator height if the elevator is already at its goal
                new ParallelDeadlineGroup(
                    searchForPeg(searchSpeed,0.0,0.0,speedRequest,false),
                    elevSub.moveElevator().onlyIf(() -> elevSub.elevatorAtHeight(height))
                    
                )
            ),

            // SCORE!!!
            new ParallelDeadlineGroup(
                coralSub.spitCoral(),
                elevSub.moveElevator()
            )

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

    public Command goToSource(Pose2d robotPose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            if (robotPose.getY() > 4.03352) {
                //Past the halfway point, go to top source, red side
                return new allign(driveSub, new Translation2d(Units.inchesToMeters(17.6),0.0), 2, 0);
            }
            else {
                //below halfway point, go to bottom source, red side
                return new allign(driveSub, new Translation2d(Units.inchesToMeters(17.6),0.0), 1, 0);
            }
        } else {
            if (robotPose.getY() > 4.03352) {
                //top source, blue side
                return new allign(driveSub, new Translation2d(Units.inchesToMeters(17.6),0.0), 13, 0);
            }
            else {
                //bottom source, blue side.
                return new allign(driveSub, new Translation2d(Units.inchesToMeters(17.6),0.0), 12, 0);
            }
        }
    }

    public Command searchForPeg(double searchSpeed, double ySpeed, double rSpeed, SwerveRequest.RobotCentric speedRequest, boolean flippingLogic){

        if (flippingLogic) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                /* */
                if (driveSub.getState().Pose.getRotation().getDegrees() > 90 || driveSub.getState().Pose.getRotation().getDegrees() < -90) {
                    return
                    //On red alliance facing blue alliance -> don't invert controls
                    driveSub.applyRequest(
                        () -> speedRequest
                            .withVelocityY(-searchSpeed)
                            .withVelocityX(ySpeed)
                            .withRotationalRate(rSpeed*0.2)
                        )
                        .until(()-> coralSub.allignedWithPeg());
                }
                else {
                    return
                    //on red alliance facing red alliance -> invert controls
                    driveSub.applyRequest(
                        () -> speedRequest
                            .withVelocityY(-searchSpeed)
                            .withVelocityX(ySpeed)
                            .withRotationalRate(rSpeed*0.2)
                        )
                        .until(()-> coralSub.allignedWithPeg());

                }
            }
            else {
                if (driveSub.getState().Pose.getRotation().getDegrees() > 90 || driveSub.getState().Pose.getRotation().getDegrees() < -90) {
                    return
                    //On blue alliance facing red alliance -> don't invert controls
                    driveSub.applyRequest(
                        () -> speedRequest
                            .withVelocityY(-searchSpeed)
                            .withVelocityX(ySpeed)
                            .withRotationalRate(rSpeed*0.2)
                        )
                        .until(()-> coralSub.allignedWithPeg());
                }
                else {
                    return
                    //on blue alliance facing blue alliance -> invert controls
                    driveSub.applyRequest(
                        () -> speedRequest
                            .withVelocityY(-searchSpeed)
                            .withVelocityX(ySpeed)
                            .withRotationalRate(rSpeed*0.2)
                        )
                        .until(()-> coralSub.allignedWithPeg());

                }
            }
        }
        else {
            return
                driveSub.applyRequest(
                    () -> speedRequest
                        .withVelocityY(searchSpeed)
                        .withVelocityX(ySpeed)
                        .withRotationalRate(rSpeed*0.2)
                ).until(()-> coralSub.allignedWithPeg());
        }


    }
}