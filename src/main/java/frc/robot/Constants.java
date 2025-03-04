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

    public final static class ElevatorConstants {
        public static class ElevatorProfiledPID {
            public static double P = 0.05;
            public static double I = 0;
            public static double D = 1.5;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        public static double maxChassisHeight = 56.5;       //inches
        public static double gearCircumference = 5.50093*2; //inches
        public static double chassisHome = 1.25; //inches

        public static double gearRatio = 1/9;
        public static int encoderID = 20;
        public static double encoderOffset = 0.0;//0.151209;
        public static int digMotorID = 21;
        public static int nanMotorID = 22;

        public static class Positions {
            public static double home = chassisHome+1;
            public static double L1 = 10;
            public static double L2 = 30;
            public static double L3 = 40;
            public static double L4 = 50;
            public static double deAlgifyL2 = 0.0;
            public static double deAlgifyL3 = 0.0;
        }
    }

    public final static class AlgaeAffectorConstants {
        public static class PivotPID {
            public static double P = 0.005;
            public static double I = 0.0;
            public static double D = 0.0009;
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
            public static double groundIntake = 0.0;
            public static double deAlgifying = 0.0;
            public static double home = 0.0;
        }
    }

    public final class CoralAffectorConstants {
        public static int coralRollerID = 43;
        public static int beamUpper = 8;
        public static int beamLower = 9;
        public static int proxSensor = 6;
    }

    public final class ClimberConstants {
        public static int digMotorID = 51;
        public static int nanMotorID = 52;
        public static int servoID = 53;
        public static double servoOpen = 0.0;
        public static double servoClosed = 0.0;
    }
}
