package frc.robot.automation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeMotor;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Otomatik skor için hazır bir sequence.
 * Hedef pozisyona gider, setpointe ulaşmayı bekler, topu dışarı atar ve HOME'a döner.
 */
public final class AutomatedScoring {
    private AutomatedScoring() {
    }

    /**
     * Skor otomasyon sequence'i.
     *
     * @param targetPosition Hedef pozisyon (örn: INTAKE_POSITION)
     * @param lift           IntakeSubsystem (yükseklik kontrolü)
     * @param roller         IntakeMotor (top içeri/dışarı)
     * @param ejectSpeed     Dışarı atma hızı (negatif değer dışarı iter)
     * @param ejectSeconds   Dışarı atma süresi
     */
    public static Command score(double targetPosition, IntakeSubsystem lift, IntakeMotor roller, double ejectSpeed,
            double ejectSeconds) {
        return new SequentialCommandGroup(
                lift.goToSetpointCommand(targetPosition),
                new WaitUntilCommand(lift::atSetpoint).withTimeout(2.5),
                roller.intakeOutCommand(Math.abs(ejectSpeed)).withTimeout(ejectSeconds),
                new WaitCommand(0.1),
                lift.goToSetpointCommand(IntakeConstants.HeightSetpoints.HOME));
    }
}
