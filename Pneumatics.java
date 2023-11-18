// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private static Compressor m_pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public static void enableCompressor() {
    m_pcmCompressor.enableDigital();
  }

  public static DoubleSolenoid addDoubleSolenoid(int channel1, int channel2) {
    return new DoubleSolenoid(PneumaticsModuleType.CTREPCM, channel1, channel2);
  }

  public static void setForwards(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public static void setBackwards(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
