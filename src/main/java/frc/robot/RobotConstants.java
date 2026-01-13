package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
public class RobotConstants {
       public static final class IntakeConstants { // TODO elevator settings
                public static final double INTAKE_MAX_HEIGHT = -120;
                public static final double INTAKE_MIN_HEIGHT = 1;

                public static final class HeightSetpoints {

                        public static final double HOME = -1;
                        public static final double HP = -50;

                        public static final class intake {
                                public static final double L0 = -30;

                                public static final double L1 = -35;
                                public static final double L2 = -74;
                                public static final double L3 = -105;
                        }

                        

                }

                public static final double MAX_MOTOR_RPM = 100000;
                public static final double MAX_MOTOR_ACCELERATION = 6500.0;

               
        }
        public static interface PortConstants {

                public static class CAN {
                        

                        public static final int INTAKE_MOTOR_1 = 16;
                        public static final int INTAKE_MOTOR_2 = 17;

                        

                }
            }
}
