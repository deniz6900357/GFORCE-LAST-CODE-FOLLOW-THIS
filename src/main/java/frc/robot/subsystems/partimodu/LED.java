package frc.robot.subsystems.partimodu;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.PWM;

public class LED extends SubsystemBase {
    private final Spark blinkin;

    // REV Blinkin Renk Kodları (Spark PWM değerleri)
    // Dokümantasyondan alınan değerler:
    private static final double SOLID_BLUE = 0.87;   // Düz Mavi
    private static final double STROBE_WHITE = -0.05; // Yanıp Sönen Beyaz (Strobe)
    private static final double STROBE_BLUE = -0.09;  // Yanıp Sönen Mavi (Strobe)

    public LED() {
        // Blinkin, PWM üzerinden Spark motor sürücüsü gibi kontrol edilir
        blinkin = new Spark(PWM.BLINKIN_PORT);
    }

    // Düz Mavi (Varsayılan)
    public void setSolidBlue() {
        blinkin.set(SOLID_BLUE);
    }

    // Yanıp Sönen Beyaz (Intake)
    public void setBlinkWhite() {
        blinkin.set(STROBE_WHITE);
    }

    // Yanıp Sönen Mavi (Atış)
    public void setBlinkBlue() {
        blinkin.set(STROBE_BLUE);
    }
}