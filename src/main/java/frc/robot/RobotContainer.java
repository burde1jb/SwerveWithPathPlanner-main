// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.RotationArmCommand;
import frc.robot.commands.NewShooterCommand;
// import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutonCommands.AutonArmDownCommand;
import frc.robot.commands.AutonCommands.AutonArmUpCommand;
import frc.robot.commands.AutonCommands.AutonIndexCommand;
import frc.robot.commands.AutonCommands.AutonIntakeCommand;
import frc.robot.commands.AutonCommands.AutonIntakeCommandLong;
import frc.robot.commands.AutonCommands.AutonPassOffCommand;
//import frc.robot.commands.AutonCommands.AutonServoCommand;
import frc.robot.commands.AutonCommands.AutonShooterCommand;
import frc.robot.commands.AutonCommands.AutonSysIdDynamicForward;
import frc.robot.commands.AutonCommands.AutonSysIdQuasistaticForward;
import frc.robot.commands.AutonCommands.AutonTiltLongCommand;
import frc.robot.commands.AutonCommands.AutonTiltShortCommand;
import frc.robot.commands.AutonCommands.AutonTimedIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;
import frc.robot.subsystems.RotationArmSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 8.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

      /* Controllers */
    public final Joystick driver = new Joystick(0);
    //private final XboxController driver2 = new XboxController(1);
  /* Setting up bindings for necessary control of the swerve drive platform */
  //private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final XboxController driver2 = new XboxController(1);
  private final IntakeSubsystem iSub;
  private final WristSubsystem wSub;
  private final VisionSubsystem vSub;
  private final RotationArmSubsystem rSub;
  private final ClimberSubsystem cSub;
  // private final ShooterSubsystem sSub;
  private final NewShooterSubsystem nsSub;
  private final LEDSubsystem lSub;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX(driver.getRawAxis(4) * MaxSpeed) // Drive forward withnegative Y (forward)
        .withVelocityY(-driver.getRawAxis(3) * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-driver.getRawAxis(0) * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ).ignoringDisable(true)); // reset the field-centric heading on left bumper press

    //driver.getRawButton(13).applyRequest(() -> brake);
    //driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    drivetrain.registerTelemetry(logger::telemeterize);

    //driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
   // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    
    this.iSub = new IntakeSubsystem();
    this.wSub = new WristSubsystem();
    this.vSub = new VisionSubsystem();
    this.rSub = new RotationArmSubsystem();
    this.cSub = new ClimberSubsystem();
    // this.sSub = new ShooterSubsystem();
    this.nsSub = new NewShooterSubsystem();
    this.lSub = new LEDSubsystem();

    NamedCommands.registerCommand("AutonArmDownCommand", new AutonArmDownCommand(rSub, wSub));
    NamedCommands.registerCommand("AutonArmUpCommand", new AutonArmUpCommand(rSub, wSub));
    NamedCommands.registerCommand("AutonIntakeCommand", new AutonIntakeCommand(iSub));
    NamedCommands.registerCommand("AutonIntakeCommandLong", new AutonIntakeCommandLong(iSub));
    NamedCommands.registerCommand("AutonTimedIntakeCommand", new AutonTimedIntakeCommand(iSub));
    NamedCommands.registerCommand("AutonShooterCommand", new AutonShooterCommand(nsSub));
    // NamedCommands.registerCommand("AutonServoCommand", new AutonServoCommand(sSub));
    NamedCommands.registerCommand("AutonIntakeOn", AutonIntakeOn());
    NamedCommands.registerCommand("AutonIntakeOff", AutonIntakeOff());
    NamedCommands.registerCommand("AutonShoot", AutonShoot());
    NamedCommands.registerCommand("AutonShootAmp", AutonShootAmp());
    NamedCommands.registerCommand("AutonShootOff", AutonShootOff());
    NamedCommands.registerCommand("AutonIndexOff", AutonIndexOff());
    NamedCommands.registerCommand("AutonSysIdDynamic", new AutonSysIdDynamicForward(drivetrain));
    NamedCommands.registerCommand("AutonSysIdStatic", new AutonSysIdQuasistaticForward(drivetrain));
    NamedCommands.registerCommand("AutonPassOff", new AutonPassOffCommand(iSub, nsSub, wSub, rSub));
    NamedCommands.registerCommand("AutonIndex", new AutonIndexCommand(nsSub));
    NamedCommands.registerCommand("AutonIndexMax", AutonIndexMax());
    NamedCommands.registerCommand("AutonTiltLongCommand", new AutonTiltLongCommand(nsSub));
    NamedCommands.registerCommand("AutonTiltShortCommand", new AutonTiltShortCommand(nsSub));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    iSub.setDefaultCommand(new IntakeCommand(iSub, driver2, driver));
    // vSub.setDefaultCommand(new VisionCommand);
    rSub.setDefaultCommand(new RotationArmCommand(rSub, wSub, driver2));
    cSub.setDefaultCommand(new ClimberCommand(cSub, driver2));
    // sSub.setDefaultCommand(new ShooterCommand(sSub, driver2));
    nsSub.setDefaultCommand(new NewShooterCommand(nsSub, driver2, driver));
    lSub.setDefaultCommand(new LEDCommand(lSub, driver2, nsSub, iSub));

    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public Command AutonIntakeOn()
  {
    return new InstantCommand(()->{iSub.intakeOn(true);});
  }

  public Command AutonIntakeOff()
  {
    return new InstantCommand(()->{iSub.intakeOff();});
  }

  public Command AutonShoot()
  {
    return new InstantCommand(()->{nsSub.newshooterOn();});
  }

  public Command AutonShootAmp()
  {
    return new InstantCommand(()->{nsSub.newshooterOn();});
  }

  public Command AutonShootOff()
  {
    return new InstantCommand(()->{nsSub.newstopshooter();});
  }

  public Command AutonIndexMax()
  {
    return new InstantCommand(()->{nsSub.indexMaxAuton(true);});
  }

  public Command AutonIndexOff()
  {
    return new InstantCommand(()->{nsSub.indexOff();});
  }
}
