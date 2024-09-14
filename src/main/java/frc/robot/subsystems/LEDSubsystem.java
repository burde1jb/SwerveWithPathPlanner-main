package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    Spark LED;

    public LEDSubsystem()   {
        LED = new Spark(1);
    }

    public void set(double powerInput)     {
        LED.set(powerInput);
    }
}