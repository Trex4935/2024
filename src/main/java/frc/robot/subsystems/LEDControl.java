// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.extension.NoteState;

public class LEDControl extends SubsystemBase {

  // Declaring ledstrip
  int m_rainbowFirstPixelHue;
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  boolean ledToggle;
  int counter;
  // Makes a new state for the shooter

  /** Creates a new Shooter. */
  public LEDControl() {

    // Creating addressable led Objects
    m_rainbowFirstPixelHue = 0;
    ledStrip = new AddressableLED(7);
    ledBuffer = new AddressableLEDBuffer(60);
    ledStrip.setLength(ledBuffer.getLength());
    ledToggle = false;
    counter = 0;
    RobotContainer.noteLifecycle = NoteState.FIELD;


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
  public void sectionedLEDControl(){
    for (var i = 0; i< ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i,Color.kBlue);
      if(i < ledBuffer.getLength()-1){
          System.out.println("+1: "+(i+1));
          ledBuffer.setLED(i+1,Color.kBlack);
      }
      if(i >= 1){
        System.out.println("-1: "+(i-1));
        ledBuffer.setLED(i-1,Color.kBlack);
      }
      if(i==0){
        ledBuffer.setLED(89,Color.kBlack);
      }

    Timer.delay(.05); 
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    }

  }
  public void solidLEDS(int hue, int saturation, int brightness){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value
      ledBuffer.setHSV(i, hue, saturation, brightness);
    }
    // Increase by to make the rainbow "move"
    // Check bounds
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }
  
  public void flashLEDS(int hue, int saturation, int brightness){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value
      if (i%2 == 0){
        if (ledToggle)
        {
          ledBuffer.setHSV(i, hue, saturation, brightness);
        }
        else
        {
          ledBuffer.setHSV(i, 0, 0, 0);
        }
      }

      else
      {
      if (ledToggle)
      {
      ledBuffer.setHSV(i, 0, 0, 0);
      }
      else
      {
        ledBuffer.setHSV(i, hue, saturation, brightness);
      }
      }
    }
    ledToggle = !ledToggle;
    // Increase by to make the rainbow "move"
    // Check bounds
    Timer.delay(5);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    
  }


  @Override
  public void periodic() {
  switch (RobotContainer.noteLifecycle) {

    case FIELD:
      counter++;
      if(counter%100 == 0)
      {
      if(!ledToggle)
      {
      solidLEDS(175, 255, 32);
      ledToggle = true;
      }
      else
      {
        solidLEDS(0, 0, 0);
        ledToggle = false;
      }
      }
      break;

    case GROUNDINTAKE:
      solidLEDS(0, 0, 32);
      break;
  
    case HUMANINTAKE:
      solidLEDS(15, 255, 32);
      break;

    case GRABBED:
      solidLEDS(5, 255, 32);
      break;

    case CONTROL:
      solidLEDS(5, 255, 32);
      break;

    case STORAGE:
      solidLEDS(250, 255, 32);
      break;

    case SPEAKER:
      solidLEDS(220, 255, 32);
      break;

    case AMPLOADING:
      solidLEDS(150, 255, 32);
      break;

    case AMP:
      solidLEDS(150, 255, 32);
      break;

    case EJECT:
      solidLEDS(1, 255, 32);
      break;

    default:
      solidLEDS(175, 255, 32);
      break;
  }

  }
  


}
