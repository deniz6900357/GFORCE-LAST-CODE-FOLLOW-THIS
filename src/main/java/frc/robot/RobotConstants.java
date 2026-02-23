package frc.robot;

public class RobotConstants {
       // Robot loop period (seconds)
       public static final double LOOP_PERIOD_SECS = 0.02; // 20ms / 50Hz
                // MAXMotion profil ayarları
                public static final double MAX_MOTOR_RPM = 100000;           // Maksimum hız (RPM)
                public static final double MAX_MOTOR_ACCELERATION = 6500.0;  // Maksimum ivme (RPM/s)

        public static interface PortConstants {

                public static class CAN {

                        public static final int INTAKE_SAKSO_MOTOR = 18;

                        // CAN Bus isimleri
                        public static final String CANIVORE_BUS = "CANivore";
                        public static final String ROBORIO_BUS = "rio"; // veya "" (boş string)

                }

                public static class PWM {
                        public static final int BLINKIN_PORT = 0; // REV Blinkin PWM Portu (0 olarak ayarlı)
                }

            }
}
