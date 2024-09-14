package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.NewShooterSubsystem;

public class NewShooterCommand extends Command {
    NewShooterSubsystem newShooterSubsystem;

    XboxController controller2;
    Joystick controller1;

    public NewShooterCommand(NewShooterSubsystem newShooterSubsystem, XboxController controller2, Joystick controller1){

        this.newShooterSubsystem = newShooterSubsystem;
        this.controller2 = controller2;
        this.controller1 = controller1;
        addRequirements(newShooterSubsystem);
    }

    @Override
    public void execute()   {

        if (controller2.getLeftTriggerAxis() > 0.2) {
            newShooterSubsystem.newshooterOn();
        }

        else {
            newShooterSubsystem.newstopshooter();
        }

        if (controller2.getRightBumper())    {
            newShooterSubsystem.indexOn();
        }
        else if (controller2.getBackButton())   {
            newShooterSubsystem.indexOut();
        }
        else if (controller2.getRightTriggerAxis() > 0.2) {
            newShooterSubsystem.indexMaxAuton(true);
        }
        else {
            newShooterSubsystem.indexOff();
        }

        if (controller2.getLeftX() > 0.2){
            newShooterSubsystem.goToshot(RobotConstants.longshot);
        }
        else if (controller2.getLeftX() < -0.2){
            newShooterSubsystem.goToshot(RobotConstants.shortshot);
        }
        else{
            newShooterSubsystem.tiltstop();
        }

        if (controller1.getRawButton(13)) {
            newShooterSubsystem.gyroreset();
        }
            
    }
}