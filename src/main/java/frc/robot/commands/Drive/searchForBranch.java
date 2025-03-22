package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.coralAffector;

/*
 *     public Command searchForPeg(double searchSpeed, double ySpeed, double rSpeed, SwerveRequest.RobotCentric speedRequest){
        return
            driveSub.applyRequest(
                () -> speedRequest
                    .withVelocityY(searchSpeed)
                    .withVelocityX(ySpeed)
                    .withRotationalRate(rSpeed*0.2)
            ).until(()-> coralSub.allignedWithPeg());
    }
 */

public class searchForBranch extends Command {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private Drive drvSub;
    private coralAffector crlSub;
    private SwerveRequest.RobotCentric speedBuilder = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private boolean leftBranch;

    private Pose2d branchPose;

    /**
     * 
     * @param drive
     * @param coral 
     * @param leftBranch True = Left Branch, False = Right Branch
     * @return A command, stupid ahh
     */
    public searchForBranch(Drive drive, coralAffector coral, boolean useLeftBranch) {
        drvSub = drive;
        crlSub = coral;
        leftBranch = useLeftBranch;
        addRequirements(drvSub,crlSub);
    }

    @Override
    public void initialize() {
        /*
        Translation2d branchOffset = (leftBranch ? new Translation2d(-0.2,0.0) : new Translation2d(0.2,0.0));
        Pose2d tagPose = drvSub.getClosestReefFace();
        Transform2d correctedOffset = new Transform2d(branchOffset.rotateBy(tagPose.getRotation()), tagPose.getRotation());
        branchPose = tagPose.plus(correctedOffset);
        */
    }

    @Override
    public void execute() {
        
        //Transform2d poseError = drvSub.getState().Pose.minus(branchPose);

        //double xError = poseError.getX();
        //double yError = poseError.getY();
        double xSpeed = 0;
        double ySpeed = 0;

        //Determine X direction
        if (!crlSub.allignedWithPeg()) {
            if (drvSub.getSensorDig()) {xSpeed = -DriveConstants.searchingSpeed;}
            if (drvSub.getSensorNan()) {xSpeed =  DriveConstants.searchingSpeed;}
        }
        else {
            xSpeed = 0;
        }


        if (!(drvSub.getSensorDig() && drvSub.getSensorNan())) {
            ySpeed = 0.1;
        }

        //Determine Y
        if (crlSub.allignedWithPeg() && (drvSub.getSensorDig() || drvSub.getSensorNan())) {
            ySpeed = 0;
        }


        drvSub.setControl(
            speedBuilder
                .withVelocityY(xSpeed)
                .withVelocityX(ySpeed)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
