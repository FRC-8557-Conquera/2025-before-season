package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.wpilibj2.command.Command;

public class resetCancoders extends Command {
  private final int[] cancoderIDs = {51, 52, 61, 62};

  @Override
  public void initialize() {
    System.out.println("🔧 Başlatılıyor: Tüm CANCoder'lar sıfırlanıyor...");

    for (int id : cancoderIDs) {
      try {
        CANcoder cancoder = new CANcoder(id);

        // 1️⃣ Fabrika ayarlarına dön
        CANcoderConfiguration factoryDefaults = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(factoryDefaults);

        // 2️⃣ Mevcut konumu 0° olarak ayarla
        MagnetSensorConfigs sensorConfig = new MagnetSensorConfigs();
        sensorConfig.MagnetOffset = 0.0; // sıfırdan başla
        cancoder.getConfigurator().apply(sensorConfig);

        // 3️⃣ Mevcut konumu fiziksel olarak "0°" kabul et
        cancoder.setPosition(0);

        System.out.println("✅ CANCoder ID " + id + " sıfırlandı.");
      } catch (Exception e) {
        System.out.println("⚠️ Hata: CANCoder ID " + id + " -> " + e.getMessage());
      }
    }

    System.out.println("🎯 Tüm CANCoder'lar başarıyla sıfırlandı!");
  }

  @Override
  public boolean isFinished() {
    return true; // tek seferlik
  }
}
