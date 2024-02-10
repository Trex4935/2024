// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDControl extends SubsystemBase {

  // Declaring ledstrip
  int m_rainbowFirstPixelHue;
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  // Makes a new state for the shooter

  /** Creates a new Shooter. */
  public LEDControl() {

    // Creating addressable led Objects
    m_rainbowFirstPixelHue = 0;
    ledStrip = new AddressableLED(7);
    ledBuffer = new AddressableLEDBuffer(90);
    ledStrip.setLength(ledBuffer.getLength());
    // Set the data
    
  }
    
  // makes motors spin YIPPIE
  public void LEDController() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 32);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }
  

  // state machine for shooter motors
  public void shooterSwitch() {
    switch (RobotContainer.noteLifecycle) {

    }

  }

  @Override
  public void periodic() {
    LEDController();
    // This method will be called once per scheduler run

  }

}
