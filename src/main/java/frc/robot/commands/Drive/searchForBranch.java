package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

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

    /**
     * 
     * @param drive
     * @param coral 
     * @param leftBranch True = Left Branch, False = Right Branch
     * @return A command, stupid ahh
     */
    public searchForBranch(Drive drive, coralAffector coral) {
        drvSub = drive;
        crlSub = coral;
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
        
        double xSpeed = 0;
        //Determine X direction
        if (!crlSub.allignedWithPeg()) {
            if (drvSub.getSensorDig()) {xSpeed =   DriveConstants.searchingSpeed;}
            if (drvSub.getSensorNan()) {xSpeed =  -DriveConstants.searchingSpeed;}
        }
        else {
            xSpeed = 0;
        }

        drvSub.setControl(
            speedBuilder
                .withVelocityY(xSpeed * MaxSpeed)
                .withVelocityX(0.0 * MaxSpeed)
                .withRotationalRate(0.0)//-Math.PI/8)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drvSub.setControl(
            speedBuilder
                .withVelocityY(0.0)
                .withVelocityX(0.25 * MaxSpeed)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        return crlSub.allignedWithPeg() && (drvSub.getSensorDig() || drvSub.getSensorNan());
    }

}
