package frc.robot;

public class RobotConstants {
       public static final class IntakeConstants {
                // Motor redüksiyon oranları
                public static final double INTAKE_GEAR_RATIO = 27.0;  // Intake lift motorları: 27:1
                public static final double ROLLER_GEAR_RATIO = 10.0;  // Sakso (roller) motor: 10:1

                // Encoder limitleri (motor tur sayısı cinsinden)
                // Not: Motor encoder değeri, redüksiyondan ÖNCE ölçülür
                public static final double INTAKE_MAX_HEIGHT = -120; // En aşağı pozisyon (motor tur)
                public static final double INTAKE_MIN_HEIGHT = 1;    // En yukarı pozisyon (motor tur)

                public static final class HeightSetpoints {
                        // Tüm değerler motor encoder tur sayısı cinsinden
                        // Çıkış mili = motor_tur / 27
                        public static final double HOME = -1;             // Ana pozisyon (-1 motor tur = -0.037 çıkış tur)
                        public static final double INTAKE_POSITION = -30; // Intake pozisyonu (-30 motor tur = -1.11 çıkış tur)
                }

                // MAXMotion profil ayarları
                public static final double MAX_MOTOR_RPM = 100000;           // Maksimum hız (RPM)
                public static final double MAX_MOTOR_ACCELERATION = 6500.0;  // Maksimum ivme (RPM/s)
        }
        public static interface PortConstants {

                public static class CAN {

                        public static final int INTAKE_MOTOR_1 = 16;
                        public static final int INTAKE_MOTOR_2 = 17;
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
