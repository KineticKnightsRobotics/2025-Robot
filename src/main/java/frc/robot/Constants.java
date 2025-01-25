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
            public static double P = 0.075;
            public static double I = 0;
            public static double D = 0.5;
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



}
