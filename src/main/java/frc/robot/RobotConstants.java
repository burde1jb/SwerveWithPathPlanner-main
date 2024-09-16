package frc.robot;

public class RobotConstants {
    
    // Roation Arm Constants
    //CAN, PWM, DIO values for motors, encoders, and sensors
    public final static int armmotorCANid = 23;
    public final static int armEncoderDIOid = 0;

    //PID Values for Motors
    public final static double armmotorP = 0;
    public final static double armmotorI = 0;
    public final static double armmotorD = 0;
    public final static double armmotorFF = 0;

    //Values for encoders - Offsets used to make end values between 0 and 1
    public final static double encoderOffset = 0.20;
    public final static double rangeOffset = 0.015;

    //Power values for motors
    public final static double goUPpower = 0.90;
    public final static double goDOWNpower = 0.95;

    public final static double dumpArmValue = 0.1980;
    public final static double dumpArmValueAuton = 0.1850;
    public final static double ampArmValue = 0.938;
    public final static double groundArmValue = 0.240;
    public final static double sourceArmValue = 0.062;

    // Wrist Arm Constants
    //CAN, PWM, DIO values for motors, encoders, and sensors
    public final static int wristmotorCANid = 25;
    public final static int wristEncoderDIOid = 1;

    //PID Values for Motors
    public final static double wristmotorP = 0;
    public final static double wristmotorI = 0;
    public final static double wristmotorD = 0;
    public final static double wristmotorFF = 0;

    //Values for encoders - Offsets used to make end values between 0 and 1
    public final static double wristencoderOffset = 0.30;
    public final static double wristrangeOffset = 0.015;

    //Power values for motors
    public final static double wristgoUPpower = -0.20;
    public final static double wristgoDOWNpower = 0.20;

    //Encoder values for set positions
    //dump = place into shooter, amp = score into amp, ground = intake from floor, source = accept from source
    public final static double dumpWristValue = 0.438;
    public final static double dumpWristExtreme = 0.263;

    public final static double ampWristValue = 0.35;
    public final static double ampWristExtreme = 0.54;

    public final static double groundWristValue = 0.631;
    public final static double groundWristExtreme = 0.263;

    public final static double sourceWristValue = 0.52;
    public final static double sourceWristExtreme = 0.26;

    //Shooter Constants
    //CAN, PWM, DIO values for motors, encoders, and sensors
    // public final static int shooter1CANid = 20;
    // public final static int shooter2CANid = 21;
    // public final static int shooterServoPWMid = 0;
    // public final static int shooterSensorDIOid = 2;

    //Power values for motors
    public final static double shooterspeed = -0.70;
    public final static double shooterServospeed = 1.00;
    public final static double shooterServoReversespeed = -1.00;
    public final static double shooterServoOffspeed = 0.00;

    //Intake Constants
    public final static int intakeCANid = 22;
    public final static int intakesensorDIOid = 4;

    //PID Values for Motors
    public final static double intakemotorP = 0;
    public final static double intakemotorI = 0;
    public final static double intakemotorD = 0;
    public final static double intakemotorFF = 0;

    //Power values for motors
    public final static double intakeOnspeed = -0.40;
    public final static double intakeOutspeed = 1.00;
    public final static double intakeSlowspeed = -0.2;

    //Climber Constants
    //CAN, PWM, DIO values for motors, encoders, and sensors
    public final static int climberCANid = 40;
    public final static int climbersensorDIOid = 6;

    //Values for encoders - Offsets used to make end values between 0 and 1
    public final static double climberencoderOffset = 20.00;
    public final static double climberrangeOffset = 1.00;

    //PID Values for Motors
    public final static double climbermotorP = 0;
    public final static double climbermotorI = 0;
    public final static double climbermotorD = 0;
    public final static double climbermotorFF = 0;

    //Power values for motors
    public final static double climberUpspeed = -0.50;
    public final static double climberDownspeed = 1.00;
    public final static double climberOffspeed = 0.00;

    //LED Values
    public final static double LEDintakesensor = 0.65;
    public final static double LEDshootersensor = 0.73;
    public final static double LEDdefault = 0.57;

    //NewShooter Constants
    //CAN, PWM, DIO values for motors, encoders, and sensors
    public final static int shooter3CANid = 50;
    public final static int shooter4CANid = 51;
    public final static int shootertiltCANid = 53;
    public final static int indexerCANid = 54;
    public final static int shooterEncoderDIOid = 5;
    public final static int shooterSensorDIOid = 2;

    //Values for encoders - Offsets used to make end values between 0 and 1
    public final static double shooterencoderOffset = 0.00;
    public final static double shooterrangeOffset = 0.007;

    //Power values for motors
    public final static double newshooterspeed = 1.0;
    public final static double indexerspeed = -0.2;
    // public final static double tiltspeed = 0.1;
    public final static double tiltUppower = 0.15;
    public final static double tiltDownpower = 0.05;
    public final static double indexerspeedOut = 0.3;
    public final static double indexerMaxspeed = -1.0;

    //Goal values for encoder on new shooter
    public final static double shortshot = 0.361;
    public final static double longshot = 0.346;
    public final static double autonlongshot = 0.290;
    
//Comment for commit


}
