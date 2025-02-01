package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;

public class Constants {
    


    public final static class VisionConstants {
        public static class defaultSTD {
            public static Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
        } 

        public static class AlignmentController {
            public static class StrafeXController {
                public static double P = 0.01;
                public static double I = 0.0;
                public static double D = 0.0;
            }
            public static class StrafeYController {
                public static double P = 0.01;
                public static double I = 0.0;
                public static double D = 0.0;
            }
            public static class RotationController {
                public static double P = 0.01;
                public static double I = 0.0;
                public static double D = 0.0;
            }
        }
    }

    public final static class ElevatorConstants {
        public static class ElevatorProfiledPID {
            public static double P = 0.075;
            public static double I = 0;
            public static double D = 0.75;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        public static double maxChassisHeight = 56.5; //inches
        public static double gearCircumference = 5.50093*2; //inches
        public static double ChassisElevationOffset = 1.25;
        public static double gearRatio = 15;
        public static int encoderID = 20;
        public static double encoderOffset = 0.26708;
        public static int leaderMotorID = 21;
        public static int followMotorID = 22;
    }

    public final static class ArmConstants {
        public static class ArmProfiledPID {
            public static double P = 0.0001;
            public static double I = 0.0;
            public static double D = 0.0;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        // Encoder reading * 360 = degrees
        public static double maxPivotPos = 0.0 * 360;
        public static double minPivotPos = 0.0 * 360;
        public static double idlePosition = 0.0* 360;
        
        public static int encoderID = 34;
        public static double encoderOffset = 0.0;
        public static int leaderMotorID = 31;
        public static int followMotorID = 32;
        public static int affectorMotorID = 33;
        public static int beamBreakPort = 0;
    }

}
