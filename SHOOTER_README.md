# Hooded Shooter Subsystem

**Mechanical Advantage Team 6328 Tabanlı Implementasyon**

Bu shooter sistemi, hem flywheel hızını hem de hood açısını kontrol eder. Mechanical Advantage'ın 2026 kodundan esinlenilerek yazılmıştır.

**Önemli:** Bu robotda **iki ayrı motor** var:
- **Kraken X60:** Flywheel (shooter hızı)
- **Kraken X44:** Hood (shooter açısı)

---

## 🎯 Sistem Bileşenleri

### **Flywheel** (1x Kraken X60)
- **Motor:** CAN ID 20 (varsayılan - değiştirin)
- **Kontrol:** Dual-mode bang-bang velocity control
  - Duty Cycle Bang-Bang: Hızlı rampa için
  - Torque Current Bang-Bang: Hedefe yakın hassas kontrol için
- **Gear Ratio:** 1:1 (direct drive)

### **Hood** (1x Kraken X44)
- **Motor:** CAN ID 22 (varsayılan - değiştirin)
- **Kontrol:** Motion Magic position control
- **Açı Aralığı:** 19° - 51°
- **Gear Ratio:** 50:1 (ÖNEMLİ: Kendi robotunuz için ölçün!)

---

## 📋 Kurulum Adımları

### 1. Motor ID'lerini Yapılandırın

[ShooterConstants.java](src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java) dosyasını açın:

```java
public static final class Flywheel {
    public static final int MOTOR_ID = 20;  // DEĞİŞTİRİN - flywheel motor ID'niz
    // ...
}

public static final class Hood {
    public static final int MOTOR_ID = 22;  // DEĞİŞTİRİN - hood motor ID'niz
    // ...
}
```

### 2. CAN Bus Konfigürasyonu

CANivore kullanıyorsanız:

```java
public static final String CAN_BUS_NAME = "canivore"; // veya "rio" için RoboRIO CAN bus
```

### 3. Gear Ratio'ları Ölçün

**Hood Gear Ratio Ölçme:**
1. Hood'u manuel olarak 1 tam tur döndürün (360°)
2. Motor encoder'ın kaç rotation okuduğuna bakın
3. Bu sayıyı `Hood.GEAR_RATIO` olarak girin

```java
public static final double GEAR_RATIO = 50.0; // motor rotasyonları : hood rotasyonu
```

**Örnek:** Hood 1° döndüğünde encoder 0.139 rotation artıyorsa:
- 360° için: 0.139 × 360 = 50 rotation
- Gear ratio = 50:1

**Flywheel Gear Ratio:**
Flywheel genellikle direct drive (1:1) olur, ancak gear kullanıyorsanız ölçün.

### 4. Motor Inversion'ları Test Edin

Her iki motoru çalıştırın ve doğru yönde döndüklerini kontrol edin:

```java
// Flywheel için
public static final boolean INVERT_MOTOR = false; // Gerekirse true yapın

// Hood için
public static final boolean INVERT_MOTOR = false; // Gerekirse true yapın
```

**Test:**
- Flywheel: Pozitif command → Top atma yönünde dönmeli
- Hood: Pozitif command → Hood yukarı gitmeli (açı artmalı)

### 5. Hood'u Zero (Kalibre) Edin

**İLK ÇALIŞTIRMADA MUTLAKA YAPIN!**

1. Hood'u fiziksel olarak minimum açıya (19°) getirin
2. Robot'u enable edin
3. SmartDashboard'da "Hood: Zero" butonuna basın veya kod ile çalıştırın:

```java
hood.zeroCommand().schedule();
```

---

## 🎮 RobotContainer'a Entegrasyon

### Subsystem'leri Oluşturun

**Önemli:** MA mimarisinde Flywheel ve Hood **ayrı subsystem'ler**dir. `Shooter` utility class'ı sadece koordinasyon için factory metodlar sağlar.

```java
public class RobotContainer {
    // Shooter subsystems (AYRI subsystem'ler, MA mimarisi)
    private final Flywheel flywheel = new Flywheel(new FlywheelIOReal());
    private final Hood hood = new Hood(new HoodIOReal());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Hood'u zero etme (POV sağ)
        joystick.povRight().onTrue(hood.zeroCommand());

        // Preset shooting positions (Shooter utility kullanarak)
        joystick.a().whileTrue(
            Shooter.runPresetCommand(flywheel, hood, ShootingPreset.FENDER)
        );
        joystick.b().whileTrue(
            Shooter.runPresetCommand(flywheel, hood, ShootingPreset.MID_RANGE)
        );
        joystick.y().whileTrue(
            Shooter.runPresetCommand(flywheel, hood, ShootingPreset.FAR)
        );

        // Auto-aim using ShotCalculator (mesafeye göre otomatik ayarlama)
        joystick.rightBumper().whileTrue(
            Shooter.runCalculatedShotCommand(flywheel, hood, drivetrain::getPose)
        );

        // Idle/stop
        joystick.x().onTrue(Shooter.idleCommand(flywheel, hood));

        // Manuel kontrol (opsiyonel)
        joystick.leftBumper().whileTrue(
            Shooter.runCustomCommand(flywheel, hood, 300.0, Math.toRadians(25))
        );
    }
}
```

### Otomatik Shot Sequence

#### Preset Kullanarak

```java
public Command autoShootSequence() {
    return Commands.sequence(
        // 1. Shooter'ı hazırla (flywheel + hood) ve bekle
        Shooter.prepareToShootCommand(flywheel, hood, ShootingPreset.MID_RANGE),

        // 2. Top besle
        feeder.feedCommand().withTimeout(0.5),

        // 3. Idle'a dön
        Shooter.idleCommand(flywheel, hood)
    );
}
```

#### ShotCalculator Kullanarak (Otomatik Mesafe Hesaplama)

```java
public Command autoAimShoot() {
    return Commands.sequence(
        // 1. Mesafeye göre hesapla ve hazırla
        Shooter.prepareCalculatedShotCommand(flywheel, hood, drivetrain::getPose),

        // 2. Top besle
        feeder.feedCommand().withTimeout(0.5),

        // 3. Idle'a dön
        Shooter.idleCommand(flywheel, hood)
    );
}
```

---

## 🎯 ShotCalculator - Otomatik Mesafe Hesaplama

**Mechanical Advantage'ın en güçlü özelliği!** ShotCalculator, robot'un pozisyonuna göre optimal hood açısı ve flywheel hızını otomatik hesaplar.

### Nasıl Çalışır?

1. **Interpolation Tabloları:** Test edilerek oluşturulan mesafe → değer tabloları
2. **Dinamik Hesaplama:** Robot pozisyonundan speaker'a mesafe hesaplanır
3. **Otomatik Ayarlama:** Hood açısı ve flywheel hızı interpolation ile bulunur

### Interpolation Tablolarını Tuning Etme

[ShotCalculator.java](src/main/java/frc/robot/subsystems/shooter/ShotCalculator.java) dosyasını açın:

```java
static {
    // Hood angle interpolation table (distance -> angle)
    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    // ...daha fazla değer ekleyin

    // Flywheel speed interpolation table (distance -> rad/s)
    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    // ...daha fazla değer ekleyin
}
```

### Tuning Süreci

1. **Test Mesafelerini Belirleyin:**
   - Speaker'dan 1.34m, 2.0m, 3.0m, 4.0m, 5.6m mesafeler işaretleyin

2. **Her Mesafe İçin Test Edin:**
   ```java
   // SmartDashboard'dan manuel ayarlama
   SmartDashboard.putNumber("Manual/Hood Angle", 25.0);
   SmartDashboard.putNumber("Manual/Flywheel Velocity", 250.0);
   ```

3. **Başarılı Değerleri Kaydedin:**
   - Shot başarılı olduğunda hood açısını ve flywheel hızını not alın
   - Bu değerleri `shotHoodAngleMap` ve `shotFlywheelSpeedMap`'e ekleyin

4. **Interpolation Test Edin:**
   - Ara mesafelerden test edin (örn: 2.5m)
   - Interpolation doğru çalışıyor mu?

### Speaker Pozisyonlarını Ayarlama

Field ölçümlerinize göre speaker pozisyonlarını güncelleyin:

```java
// Hub/speaker positions (meters)
private static final Translation2d BLUE_SPEAKER = new Translation2d(0.0, 5.55);
private static final Translation2d RED_SPEAKER = new Translation2d(16.54, 5.55);
```

### Mesafe Limitleri

Güvenli atış mesafe aralığını ayarlayın:

```java
private static final double MIN_DISTANCE = 1.34; // minimum mesafe (meter)
private static final double MAX_DISTANCE = 5.60; // maximum mesafe (meter)
```

### Telemetry

ShotCalculator telemetry ile debug yapın:

- `ShotCalc/Distance (m)` - Speaker'a mesafe
- `ShotCalc/Hood Angle (deg)` - Hesaplanan hood açısı
- `ShotCalc/Flywheel Velocity (rad/s)` - Hesaplanan flywheel hızı
- `ShotCalc/In Range` - Mesafe geçerli mi?
- `ShotCalc/Status` - READY / OUT OF RANGE

---

## ⚙️ Tuning

### Flywheel Tuning

#### Velocity Tolerance
Flywheel'in hedefe ne kadar yakın olması gerektiğini ayarlar:

```java
public static final double VELOCITY_TOLERANCE = 20.0; // rad/s (~3.2 RPS)
```

#### Torque Current Scale
Hedefe yakın hassas kontrol için akım miktarı:

```java
public static final double TORQUE_CURRENT_SCALE = 2.0; // Amps per rad/s
```

**Tuning İpuçları:**
- Flywheel hedefe ulaşamıyorsa → `TORQUE_CURRENT_SCALE` artırın
- Flywheel overshoot yapıyorsa → `TORQUE_CURRENT_SCALE` azaltın
- "At Goal" çok geç tetikleniyorsa → `VELOCITY_TOLERANCE` artırın

### Hood Tuning

#### Motion Magic Ayarları

[HoodIOReal.java](src/main/java/frc/robot/subsystems/shooter/hood/HoodIOReal.java) dosyasında:

```java
config.MotionMagic.MotionMagicCruiseVelocity = 80.0;  // rot/s (azaltın: daha yavaş)
config.MotionMagic.MotionMagicAcceleration = 160.0;   // rot/s^2
config.MotionMagic.MotionMagicJerk = 1600.0;          // rot/s^3
```

#### PID Gains

[ShooterConstants.java](src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java) dosyasında:

```java
public static final double KP = 30000.0; // Overshooting varsa azaltın
public static final double KD = 300.0;   // Oscillation varsa artırın
```

**Tuning İpuçları:**
- Hood çok yavaş → Cruise Velocity artırın (80 → 120)
- Hood "titriyor" → kD artırın veya kP azaltın
- Hood hedefe geçiyor → kP azaltın (30000 → 20000)
- Hood hedefe varamıyor → kP artırın (30000 → 40000)

### Preset Velocities ve Angles

Test ederek en iyi değerleri bulun:

```java
public static final class Flywheel {
    public static final double FENDER_VELOCITY = 200.0;    // rad/s
    public static final double MID_RANGE_VELOCITY = 350.0;
    public static final double FAR_VELOCITY = 450.0;
}

public static final class Hood {
    public static final double FENDER_ANGLE = Math.toRadians(19.0);
    public static final double MID_RANGE_ANGLE = Math.toRadians(30.0);
    public static final double FAR_ANGLE = Math.toRadians(45.0);
}
```

---

## 📊 SmartDashboard Telemetry

### Flywheel Metrikleri
- `Flywheel/Goal Velocity (rad/s)` - Hedef hız
- `Flywheel/Measured Velocity (rad/s)` - Gerçek hız
- `Flywheel/Velocity Error (rad/s)` - Hata miktarı
- `Flywheel/RPM` - RPM cinsinden hız
- `Flywheel/Goal RPM` - Hedef RPM
- `Flywheel/At Goal` - Hedefe ulaştı mı?
- `Flywheel/Control Mode` - COAST / DUTY_CYCLE_BANG_BANG / TORQUE_CURRENT_BANG_BANG
- `Flywheel/Current (A)` - Motor akımı
- `Flywheel/Temp (C)` - Motor sıcaklığı
- `Flywheel/Connected` - Motor bağlı mı?

### Hood Metrikleri
- `Hood/Goal Angle (deg)` - Hedef açı
- `Hood/Measured Angle (deg)` - Gerçek açı (kalibre edilmiş)
- `Hood/Raw Position (deg)` - Motor encoder ham değeri
- `Hood/Offset (deg)` - Kalibrasyon offset'i
- `Hood/Angle Error (deg)` - Hata miktarı
- `Hood/At Goal` - Hedefe ulaştı mı?
- `Hood/Zeroed` - Kalibre edilmiş mi?
- `Hood/Current (A)` - Motor akımı
- `Hood/Temp (C)` - Motor sıcaklığı
- `Hood/Connected` - Motor bağlı mı?
- `Hood/Control Mode` - BRAKE / COAST / CLOSED_LOOP

---

## 🚀 Kullanım Örnekleri

### Basit Preset Kullanımı

```java
// Fender shot (yakın mesafe)
joystick.a().whileTrue(
    Shooter.runPresetCommand(flywheel, hood, ShootingPreset.FENDER)
);

// Mid-range (orta mesafe)
joystick.b().whileTrue(
    Shooter.runPresetCommand(flywheel, hood, ShootingPreset.MID_RANGE)
);

// Far shot (uzak mesafe)
joystick.y().whileTrue(
    Shooter.runPresetCommand(flywheel, hood, ShootingPreset.FAR)
);

// Idle (güvenli pozisyon)
joystick.x().onTrue(Shooter.idleCommand(flywheel, hood));
```

### Auto-Aim (ShotCalculator Kullanarak)

**En önemli özellik!** Mesafeye göre otomatik hood açısı ve flywheel hızı hesaplama:

```java
// Dinamik otomatik hedef alma - robot pozisyonuna göre sürekli ayarlama
joystick.rightBumper().whileTrue(
    Shooter.runCalculatedShotCommand(flywheel, hood, drivetrain::getPose)
);

// Veya hazırla ve bekle versiyonu
joystick.rightTrigger().onTrue(
    Shooter.prepareCalculatedShotCommand(flywheel, hood, drivetrain::getPose)
);
```

### Custom Kontrol

```java
// Özel hız ve açı
joystick.leftBumper().whileTrue(
    Shooter.runCustomCommand(
        flywheel, hood,
        400.0,                    // 400 rad/s flywheel velocity
        Math.toRadians(35.0)      // 35° hood angle
    )
);
```

### Sadece Flywheel veya Sadece Hood

MA mimarisinde her subsystem bağımsız - doğrudan kullanabilirsiniz:

```java
// Sadece flywheel
joystick.rightTrigger().whileTrue(
    flywheel.runVelocityCommand(300.0)
);

// Sadece hood
joystick.leftTrigger().whileTrue(
    hood.setAngleCommand(Math.toRadians(25))
);
```

### Otomatik Shot Sequence (Feeder ile)

#### Preset ile

```java
public Command fullAutoShot(ShootingPreset preset) {
    return Commands.sequence(
        // 1. Shooter'ı hazırla ve bekle
        Shooter.prepareToShootCommand(flywheel, hood, preset),

        // 2. Flywheel ve hood hazır, şimdi besle
        feeder.feedCommand().withTimeout(0.5),

        // 3. Bitince idle'a dön
        Shooter.idleCommand(flywheel, hood)
    );
}

// Kullanımı:
joystick.a().onTrue(fullAutoShot(ShootingPreset.FENDER));
```

#### ShotCalculator ile (Otomatik Hedef Alma)

```java
public Command autoAimAndShoot() {
    return Commands.sequence(
        // 1. Mesafeye göre otomatik hesapla ve hazırla
        Shooter.prepareCalculatedShotCommand(flywheel, hood, drivetrain::getPose),

        // 2. Flywheel ve hood hazır, besle
        feeder.feedCommand().withTimeout(0.5),

        // 3. Bitince idle'a dön
        Shooter.idleCommand(flywheel, hood)
    );
}

// Kullanımı:
joystick.y().onTrue(autoAimAndShoot());
```

### Koşullu Shooting

```java
public Command smartShoot() {
    return Commands.either(
        // Yakın mesafe (<2m) - preset kullan
        Shooter.runPresetCommand(flywheel, hood, ShootingPreset.FENDER),

        // Uzak mesafe (>2m) - ShotCalculator kullan
        Shooter.runCalculatedShotCommand(flywheel, hood, drivetrain::getPose),

        // Koşul: mesafe 2m'den küçük mü?
        () -> {
            var params = ShotCalculator.getInstance().calculateShot(drivetrain.getPose());
            return params.distanceToTarget < 2.0;
        }
    );
}
```

---

## 🔧 Troubleshooting

### Problem: Flywheel hedefe ulaşamıyor

**Çözümler:**
- `TORQUE_CURRENT_SCALE` değerini artırın (2.0 → 3.0)
- Motor akımını kontrol edin (60A limiti yeterli mi?)
- Motor inversion'ı test edin
- `Flywheel/Control Mode` TORQUE_CURRENT_BANG_BANG'e geçiyor mu?

### Problem: Flywheel çok uzun süre rampa yapıyor

**Çözümler:**
- `VELOCITY_TOLERANCE` değerini kontrol edin (çok düşük olabilir)
- Supply current limit'i artırın (60A → 80A)
- Direct drive olduğundan emin olun (GEAR_RATIO = 1.0)

### Problem: Hood hareket etmiyor

**Çözümler:**
- Hood'un zero edildiğinden emin olun (`Hood/Zeroed = true`)
- Motor ID'sinin doğru olduğunu kontrol edin
- Gear ratio'nun doğru olduğunu doğrulayın
- Motor inversion'ı test edin

### Problem: Hood yanlış açıya gidiyor

**Çözümler:**
- **Gear ratio yanlış** olabilir → Tekrar ölçün
- `Hood/Raw Position` ve `Hood/Measured Angle` değerlerini karşılaştırın
- Fark 50-100x mertebesinde olmalı (gear ratio kadar)
- Hood'u tekrar zero edin

### Problem: "At Goal" hiç true olmuyor

**Çözümler:**
- Tolerance değerlerini artırın
- SmartDashboard'dan error değerlerini izleyin
- Hood için: Zero yapılmış mı kontrol edin
- Flywheel için: TORQUE_CURRENT_SCALE yeterli mi?

### Problem: Her iki motor da çalışıyor ama koordinasyon yok

**Çözümler:**
- `Shooter.readyToShoot()` metodunu kullanın
- `prepareToShootCommand()` içindeki `waitUntil()` çalışıyor mu?
- Hem `Flywheel/At Goal` hem `Hood/At Goal` true olmalı

---

## ✅ Başarı Checklist

Shooter sistemi çalışmaya hazır mı? Kontrol edin:

### Flywheel
- [ ] Motor ID doğru yapılandırıldı (varsayılan: 20)
- [ ] Motor inversion test edildi
- [ ] Flywheel full hızda dönebiliyor
- [ ] `At Goal` indikatörü çalışıyor
- [ ] `Flywheel/Connected` = true
- [ ] Preset velocities test edildi
- [ ] RPM değerleri mantıklı

### Hood
- [ ] Motor ID doğru yapılandırıldı (varsayılan: 22)
- [ ] Gear ratio ölçüldü ve girildi
- [ ] Motor inversion test edildi
- [ ] Hood zero edildi (ilk çalıştırmada)
- [ ] Hood tüm açı aralığında (19°-51°) hareket edebiliyor
- [ ] `At Goal` indikatörü çalışıyor
- [ ] `Hood/Zeroed` = true
- [ ] Preset açılar test edildi

### Kombine Sistem
- [ ] Her iki subsystem da aynı anda çalışabiliyor
- [ ] `Shooter.readyToShoot(flywheel, hood)` doğru değer döndürüyor
- [ ] Preset'ler hem flywheel hem hood'u kontrol ediyor
- [ ] Telemetry SmartDashboard'da görünüyor
- [ ] Shot sequence'lar test edildi

### ShotCalculator
- [ ] Speaker pozisyonları doğru ayarlandı
- [ ] Interpolation tabloları test edildi
- [ ] En az 5-6 farklı mesafe için değerler girildi
- [ ] `ShotCalc/Distance` doğru değer gösteriyor
- [ ] Auto-aim komutu çalışıyor
- [ ] Ara mesafeler için interpolation doğru

---

## 📂 Dosya Yapısı (MA Architecture)

```
src/main/java/frc/robot/subsystems/shooter/
├── flywheel/
│   ├── Flywheel.java          # Flywheel subsystem (velocity control)
│   ├── FlywheelIO.java        # Hardware interface
│   └── FlywheelIOReal.java    # CTRE Kraken X60 implementation
├── hood/
│   ├── Hood.java              # Hood subsystem (angle control)
│   ├── HoodIO.java            # Hardware interface
│   └── HoodIOReal.java        # CTRE Kraken X44 implementation
├── Shooter.java               # Utility class (factory methods)
├── ShotCalculator.java        # Distance-based shot calculator
└── ShooterConstants.java      # Konfigürasyon sabitleri
```

**Önemli:** MA mimarisinde:
- Flywheel ve Hood **ayrı subsystem'ler** (bağımsız çalışabilir)
- `Shooter` bir **utility class** (SubsystemBase değil)
- `ShotCalculator` mesafeye göre interpolation yapar (singleton)
- `ShooterConstants` preset değerleri içerir

---

## 🎯 Mechanical Advantage Mimarisi Hakkında

Bu kod, Team 6328 Mechanical Advantage'ın 2026 sezonundaki shooter implementasyonunu takip eder:

### IO Abstraction Pattern
- Hardware kodu subsystem'den ayrı
- Simulation ve test kolaylığı
- Real hardware ve sim aynı interface'i kullanır

### Bang-Bang Control (Flywheel)
- **Duty Cycle Mode:** Hızlı rampa için full power
- **Torque Current Mode:** Hedefe yakın hassas kontrol
- PID yerine daha basit ve etkili

### Motion Magic (Hood)
- Smooth trapezoid profili
- Acceleration ve jerk sınırlaması
- Güvenli ve tutarlı hareket

### Dual Subsystem Coordination
- Her subsystem **tamamen bağımsız** çalışır
- `Shooter` utility class'ı factory metodlar sağlar (subsystem değil!)
- `ShotCalculator` singleton - interpolation tabloları ile otomatik hesaplama
- Command-based framework ile kolay entegrasyon

### Distance-Based Shot Calculation
- **InterpolatingTreeMap:** Mesafe değerleri arasında interpolation
- **Alliance-Aware:** Blue/Red alliance'a göre speaker pozisyonu
- **Validation:** Min/max mesafe kontrolü
- **Telemetry:** Her hesaplama SmartDashboard'da görünür

---

## 📞 Destek

Sorularınız için:
1. SmartDashboard telemetry'yi kontrol edin
2. Her iki motor için `Connected` statusunu doğrulayın
3. Hood için `Zeroed` statusunu kontrol edin
4. Flywheel için `Control Mode` geçişlerini izleyin
5. Motor current ve temp değerlerini kontrol edin

**Kod Mechanical Advantage Team 6328'in 2026 açık kaynak kodundan esinlenilmiştir.**

GitHub: https://github.com/Mechanical-Advantage/RobotCode2026Public

---

Başarılar! 🎯🤖
