package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.teleop.Utilities;

@TeleOp(name="Checkpoint4")
public class Checkpoint4TeleOp extends OpMode {
    Hardware hardware;
    Utilities utilities;

    //Drivetrain speeds
    final double FAST_SPEED = 1.0;
    final double MID_SPEED = .5;
    final double SLOW_SPEED = .25;
    final double DPAD_SPEED = .25;
    double speedConstant;

    boolean fieldOriented;
    boolean intakeOn;
    boolean intakeReverseOn;
    boolean clawOpen;

    final double INTAKE_SPEED = 1;
    final double LAUNCHER_AIM_LOW_BOUND = 0;
    final double LAUNCHER_AIM_HIGH_BOUND = 1;

    double launcherAimServoPosition;

    final double INTAKE_DEPLOY1_MAX_POSITION = 0.50; // right up
    final double INTAKE_DEPLOY1_MIN_POSITION = 0.05; // right down
    final double INTAKE_DEPLOY2_MAX_POSITION = 0.65; // left down
    final double INTAKE_DEPLOY2_MIN_POSITION = 0.20; // left up
    double intakeDeployServoPosition1;
    double intakeDeployServoPosition2;

    final double FLYWHEEL_FAST_CAP = 1;
    final double FLYWHEEL_SLOW_CAP = .4;
    double flywheelSpeed;

    int armPosition;
    final double ARM_POWER = 1;

    // ElapsedTime
    ElapsedTime flywheelTime;
    ElapsedTime launcherAimTime;
    ElapsedTime sinceStartTime;
    ElapsedTime intakeTime;
    ElapsedTime intakeBackTime;
    ElapsedTime intakeDeployTime;
    ElapsedTime clawTime;

    PIDFCoefficients pidfCoefficients;
    final double kP = 20;
    final double kI = 0;
    final double kD = .1;
    final double kF = -30;
    double kFMultiplier;

    // Field oriented
    Orientation angles = new Orientation();
    double initYaw;
    double adjustedYaw;

    @Override
    public void init() {
        hardware = new Hardware();
        Assert.assertNotNull(hardwareMap);
        hardware.init(hardwareMap);

        utilities = new Utilities(hardware);

        speedConstant = FAST_SPEED;
        fieldOriented = false;
        intakeOn = false;
        clawOpen = false;
        launcherAimServoPosition = 0;
        flywheelSpeed = 1.0;
        armPosition = 0;
        hardware.arm.setTargetPosition(armPosition);
        hardware.arm.setPower(ARM_POWER);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Setup field oriented
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;

        telemetry.addData("Status:: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status:: ", "Started");
        telemetry.update();
        sinceStartTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        flywheelTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        launcherAimTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeBackTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeDeployTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        launcherAimServoPosition = 0.35;
        intakeDeployServoPosition1 = 0.05; // 0.05 drop 0.45 up
        intakeDeployServoPosition2 = 0.65; // 0.65 drop 0.25 up
        intakeReverseOn = false;

    }

    @Override
    public void loop() {
        drive();
        intake();
        launcher();
        arm();
        telemetry();

    }

    public void telemetry() {
        telemetry.addData("Flywheel velocity:: ", hardware.flywheel.getVelocity());
        telemetry.addData("Launcher servo:: ", hardware.launcherAimServo.getPosition());
        telemetry.addData("Arm position:: ", armPosition);
        telemetry.addData("Arm position actual:: ", hardware.arm.getCurrentPosition());
        telemetry.update();
    }

    public void drive() {

        // Update modes

        // Change slow mode
        if (gamepad1.cross) {
            speedConstant = SLOW_SPEED;
        } else if (gamepad1.circle) {
            speedConstant = MID_SPEED;
        } else if (gamepad1.triangle) {
            speedConstant = FAST_SPEED;
        }

        // Change field oriented mode
        if (gamepad1.share) {
            fieldOriented = true;
        } else if (gamepad1.options) {
            fieldOriented = false;
        }

        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        // Run D-pad or joystick movement controls
        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)
        {
            //D-pad controls
            if (gamepad1.dpad_up)
            {
                leftFrontPower = DPAD_SPEED;
                leftRearPower = DPAD_SPEED;
                rightFrontPower = DPAD_SPEED;
                rightRearPower = DPAD_SPEED;
            }
            else if (gamepad1.dpad_down)
            {
                leftFrontPower = -DPAD_SPEED;
                leftRearPower = -DPAD_SPEED;
                rightFrontPower = -DPAD_SPEED;
                rightRearPower = -DPAD_SPEED;
            }
            else if (gamepad1.dpad_right)
            {
                leftFrontPower = DPAD_SPEED;
                leftRearPower = -DPAD_SPEED;
                rightFrontPower = -DPAD_SPEED;
                rightRearPower = DPAD_SPEED;
            }
            else
            {
                leftFrontPower = -DPAD_SPEED;
                leftRearPower = DPAD_SPEED;
                rightFrontPower = DPAD_SPEED;
                rightRearPower = -DPAD_SPEED;
            }
        }
        else
        {
            // Mecanum drivecode
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;


            // Field oriented
            if (fieldOriented) {
                angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                adjustedYaw = angles.firstAngle-initYaw;

                double zerodYaw = -initYaw+angles.firstAngle;

                double theta = Math.atan2(y, x) * 180/Math.PI; // aka angle

                double realTheta;

                realTheta = (360 - zerodYaw) + theta;

                double power = Math.hypot(x, y);

                double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
                double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
                double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

                leftFrontPower = (power * cos / maxSinCos + turn);
                rightFrontPower = (power * sin / maxSinCos - turn);
                leftRearPower = (power * sin / maxSinCos + turn);
                rightRearPower = (power * cos / maxSinCos - turn);
            }
            else {
                leftFrontPower = y + x + turn;
                leftRearPower = y - x + turn;
                rightFrontPower = y - x - turn;
                rightRearPower = y + x - turn;
            }

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
                // Find the largest power
                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(Math.abs(rightFrontPower), max);
                max = Math.max(Math.abs(rightRearPower), max);

                // Divide everything by max (it's positive so we don't need to worry about signs)
                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            // Non-linear (Quadratic) control for finer adjustments at low speed
            leftFrontPower = Math.pow(leftFrontPower, 2) * Math.signum(leftFrontPower) * speedConstant;
            leftRearPower = Math.pow(leftRearPower, 2) * Math.signum(leftRearPower) * speedConstant;
            rightFrontPower = Math.pow(rightFrontPower, 2) * Math.signum(rightFrontPower) * speedConstant;
            rightRearPower = Math.pow(rightRearPower, 2) * Math.signum(rightRearPower) * speedConstant;
        }

        // Set motor power
        hardware.rightRear.setPower(rightRearPower);
        hardware.rightFront.setPower(rightFrontPower);
        hardware.leftFront.setPower(leftFrontPower);
        hardware.leftRear.setPower(leftRearPower);
    }

    private void intake()
    {
        // Intake power
        if(gamepad2.square && intakeTime.time() >= 250)
        {
            intakeOn = !intakeOn;
            intakeReverseOn = false;
            intakeTime.reset();
            if(intakeOn)
            {
                hardware.intake.setPower(INTAKE_SPEED);
            }
            else
            {
                hardware.intake.setPower(0.0);
            }
        }
        if(gamepad2.cross && intakeBackTime.time() >= 250)
        {
            intakeOn = false;
            intakeReverseOn = !intakeReverseOn;
            if(intakeReverseOn)
            {
                hardware.intake.setPower(-0.5);
            }
            else
            {
                hardware.intake.setPower(0);
            }
            intakeBackTime.reset();
        }

        // Intake deployment
        if(Math.abs(gamepad2.right_stick_y) >= 0.1 && intakeDeployTime.time() >= 50)
        {
            intakeDeployTime.reset();
            if(gamepad2.right_stick_y > 0)
            {
                intakeDeployServoPosition1 = Math.min(intakeDeployServoPosition1 + 0.05, INTAKE_DEPLOY1_MAX_POSITION);
                intakeDeployServoPosition2 = Math.max(intakeDeployServoPosition2 - 0.05, INTAKE_DEPLOY2_MIN_POSITION);
            }
            else
            {
                intakeDeployServoPosition1 = Math.max(intakeDeployServoPosition1 - 0.05, INTAKE_DEPLOY1_MIN_POSITION);
                intakeDeployServoPosition2 = Math.min(intakeDeployServoPosition2 + 0.05, INTAKE_DEPLOY2_MAX_POSITION);
            }
        }
        hardware.intakeDeployServo1.setPosition(intakeDeployServoPosition1);
        hardware.intakeDeployServo2.setPosition(intakeDeployServoPosition2);
    }

    private void launcher()
    {
        // Flywheel speed
        if(gamepad2.dpad_up && flywheelTime.time() >= 250)
        {
            flywheelSpeed = Math.min(flywheelSpeed + 0.10, FLYWHEEL_FAST_CAP);
            flywheelTime.reset();
        }
        else if(gamepad2.dpad_down && flywheelTime.time() >= 250)
        {
            flywheelSpeed = Math.max(flywheelSpeed - 0.10, FLYWHEEL_SLOW_CAP);
            flywheelTime.reset();
        }
        hardware.flywheel.setPower(gamepad2.left_trigger * flywheelSpeed);

        // Shoot
        if(gamepad2.right_trigger > 0.1)
        {
            utilities.shoot();
        }

        // Telescoping hood position
        if(gamepad2.dpad_right && launcherAimTime.time() >= 50)
        {
            launcherAimServoPosition = Math.min(launcherAimServoPosition + 0.025, LAUNCHER_AIM_HIGH_BOUND);
            launcherAimTime.reset();
        }
        else if(gamepad2.dpad_left && launcherAimTime.time() >= 50)
        {
            launcherAimServoPosition = Math.max(launcherAimServoPosition - 0.025, LAUNCHER_AIM_LOW_BOUND);
            launcherAimTime.reset();
        }
        hardware.launcherAimServo.setPosition(launcherAimServoPosition);
    }

    private void arm()
    {
        if(gamepad2.right_bumper)
        {
            armPosition = 125; // Junction
        }
        else if(gamepad2.left_bumper)
        {
            armPosition = 190; // Ground
        }
        else if(gamepad2.circle)
        {
            armPosition = 0; // Retracted
        }

        // Arm position
        hardware.arm.setTargetPosition(armPosition);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(ARM_POWER);
        // Claw
        if(gamepad2.triangle && clawTime.time() >= 250)
        {
            clawOpen = !clawOpen;
            utilities.clawControl(clawOpen);
        }
    }
}