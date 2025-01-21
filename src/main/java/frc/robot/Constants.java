package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;

public class Constants {
    


    public final static class VisionConstants {
        public static class defaultSTD {
            public static Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
        } 
    }

    public final static class ElevatorConstants {
        public static class ElevatorProfiledPID {
            public static double P = 0.001;
            public static double I = 0;
            public static double D = 0;
            public static double MaxVelocity = 0;
            public static double MaxAcceleration = 0;
        }

        public static double maxElevatorHeight = 28; //inches
        public static double gearCircumference = 5.53; //inches
        public static double gearRatio = 25;
        public static int encoderID = 20;
        public static double encoderOffset = 0.1044921875;//0.198739;
        public static int leaderMotorID = 21;
        public static int followMotorID = 22;
    }



}
