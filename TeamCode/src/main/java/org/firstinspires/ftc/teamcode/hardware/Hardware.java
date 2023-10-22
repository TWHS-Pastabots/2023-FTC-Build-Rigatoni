package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class Hardware {

    // Drive Motors
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx[] driveMotors;

    // Subsystem Motors
    public DcMotorEx arm;
    public DcMotorEx intake;
    public DcMotorEx flywheel;

    // Launcher Servos
    public Servo launcherAimServo;
    public Servo launcherTriggerServo;
    public Servo launcherReleaseServo;
    // Claw Servos
    public Servo clawServo;

    // IMU
    public BNO055IMU imu;

    public void init(HardwareMap hardwareMap) {
        Assert.assertNotNull(hardwareMap);
        initializeDrive(hardwareMap);
        initializeSupplementaryMotors(hardwareMap);
    }

    public void initializeDrive(HardwareMap hardwareMap) {

        // Set up drive motors
        rightRear = hardwareMap.get(DcMotorEx.class, HardwareIDs.RIGHT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareIDs.RIGHT_FRONT_MOTOR);
        leftFront = hardwareMap.get(DcMotorEx.class, HardwareIDs.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, HardwareIDs.LEFT_REAR_MOTOR);

        // Set left motors to reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{rightRear, rightFront, leftFront, leftRear};

        // Set Zero Power Behavior and Initialize Motors
        for(DcMotorEx motor: driveMotors) {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set up IMU
        imu = hardwareMap.get(BNO055IMU.class, HardwareIDs.IMU);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    public void initializeSupplementaryMotors(HardwareMap hardwareMap) {
        // Set up servos

        // Set up launcher servos
        launcherAimServo = hardwareMap.get(Servo.class, HardwareIDs.AIMING_SERVO);
        launcherTriggerServo = hardwareMap.get(Servo.class, HardwareIDs.RING_TRIGGER_SERVO);
        launcherReleaseServo = hardwareMap.get(Servo.class, HardwareIDs.LAUNCHER_RELEASE_SERVO);

        // Set up claw servos
        clawServo = hardwareMap.get(Servo.class, HardwareIDs.CLAW_SERVO);

        // Set servo positions

        // Launcher servos
        launcherAimServo.setPosition(0);
        launcherTriggerServo.setPosition(0);
        launcherReleaseServo.setPosition(0);
        // Claw servos
        clawServo.setPosition(0); //closed


        //Set up subsystem motors
        arm = hardwareMap.get(DcMotorEx.class, HardwareIDs.ARM_MOTOR);
        intake = hardwareMap.get(DcMotorEx.class, HardwareIDs.INTAKE_MOTOR);
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareIDs.FLYWHEEL_MOTOR);


        // Set zero power behavior and initialize subsystem motors
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setPower(0);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setPower(0);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}