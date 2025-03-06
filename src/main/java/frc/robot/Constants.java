package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    


    public final static class VisionConstants {
        public static class defaultSTD {
            public static Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
        } 

        public static class AlignmentController {
            public static class StrafeXController {
                public static double P = 1.5;
                public static double I = 0.0;
                public static double D = 0.15;
            }
            public static class StrafeYController {
                public static double P = 1.5;
                public static double I = 0.0;
                public static double D = 0.15;
            }
            public static class RotationController {
                public static double P = 0.1;
                public static double I = 0.0;
                public static double D = 0.0;
            }
        }
    }
    public final static class QuestConstants {
        public Translation2d headsetRobotPose = new Translation2d(0,37);
    }

    public final static class DriveConstants {
        public static double searchingSpeed = 0.35;
        public static int resetButton = 3;
    }

    public final static class ElevatorConstants {
        public static class ElevatorProfiledPID {
            public static double P = 0.1;
            public static double I = 0;
            public static double D = 0.0;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        public static double maxChassisHeight = 56.5;       //inches
        public static double minChassisHeight = 1.25;       //inches
        public static double gearCircumference = 5.50093*2; //inches


        public static double gearRatio = 1/9;
        public static int encoderID = 20;
        public static double encoderOffset = 0.0;
        public static int digMotorID = 21;
        public static int nanMotorID = 22;

        public static double tippingPoint = 10; //Height at which the robot begins to tip when driving

        public static class Positions {
            public static double home = minChassisHeight+1;
            public static double intake = home+0.5;
            public static double L1 = 4;
            public static double L2 = 17;
            public static double L3 = 34;
            public static double L4 = 57;
            public static double deAlgifyL2 = 11;
            public static double deAlgifyL3 = 28;
        }
    }

    public final static class AlgaeAffectorConstants {
        public static class PivotPID {
            public static double P = 0.0125;
            public static double I = 0.0;
            public static double D = 0.000;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }
        public static int absoluteEncoderID = 34;
        public static int pivotMotorID = 31;
        public static int rollerMotorID = 32;
        public static int proxSensor = 7;

        public static double encoderOffset = 0.74335;
        public static double algaePivotGearRatio = 1/5;

        public static class PivotPositions {
            public static double groundIntake = 10.0;
            public static double deAlgifying = 65.0;
            public static double carrying = 49;
            public static double home = 110.0;
        }
    }

    public final class CoralAffectorConstants {
        public static int coralRollerID = 43;
        public static int beamUpper = 9;
        public static int beamLower = 8;
        public static int proxSensor = 6;
    }

    public final class ClimberConstants {
        public static int digMotorID = 51;
        public static int nanMotorID = 52;
        public static int winchID = 53;
    }
}
