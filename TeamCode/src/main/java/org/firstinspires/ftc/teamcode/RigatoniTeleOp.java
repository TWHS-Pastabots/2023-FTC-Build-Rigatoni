package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="Rigatoni")
public class RigatoniTeleOp extends OpMode {
    Hardware hardware;
    Utilities utilities;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    final double FAST_SPEED = 1.0;
    final double MID_SPEED = .5;
    final double SLOW_SPEED = .25;

    final double DPAD_SPEED = .25;

    double speedConstant;
    boolean fieldOriented;
    boolean controlOverride;
    boolean intakeOn = false;
    double intakeSpeed = 1.0;
    boolean clawOpen = false;

    double armSpeed = 1;
    final double LAUNCHER_AIM_SERVO_ADJUSTMENT = 0.1;

    ElapsedTime sinceStartTime;

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
        controlOverride = false;

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
    }

    @Override
    public void loop() {
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

        drive();
        intake();
        launcher();
        arm();

    }

    public void drive() {

        // Update modes

        // Change slow mode
        if (gamepad1.cross) {
            speedConstant = SLOW_SPEED;
        } else if (gamepad1.square) {
            speedConstant = MID_SPEED;
        } else if (gamepad1.triangle) {
            speedConstant = FAST_SPEED;
        }

        // Change field oriented mode
        if (gamepad1.options) {
            fieldOriented = true;
        } else if (gamepad1.share) {
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
        if(gamepad2.square)
        {
            hardware.intake.setPower(intakeSpeed);
        }
        if(gamepad2.cross)
        {
            hardware.intake.setPower(0.0);
        }
    }

    private void launcher()
    {
        hardware.flywheel.setPower(gamepad2.left_trigger);
        if(gamepad2.right_trigger > 0.1)
        {
            utilities.shoot();
        }
        double launcherAimServoPosition = hardware.launcherAimServo.getPosition() + LAUNCHER_AIM_SERVO_ADJUSTMENT * gamepad2.right_stick_x;
        if(launcherAimServoPosition > 1.0)
        {
            launcherAimServoPosition = 1.0;
        }
        else if(launcherAimServoPosition < -1.0)
        {
            launcherAimServoPosition = -1.0;
        }
        hardware.launcherAimServo.setPosition(launcherAimServoPosition);

    }

    private void arm()
    {
        // Prevent accidental arm trigger
        if(gamepad2.options)
        {
            controlOverride = true;
        }
        // If endgame or overridden allow arm use
        if(sinceStartTime.time() > 90000 || controlOverride)
        {
            if(Math.abs(gamepad2.left_stick_y) > 0.05) //Setting deadzone for arm breaking
            {
                hardware.arm.setPower(-gamepad2.left_stick_y * armSpeed);
            }
            else
            {
                hardware.arm.setPower(0);
            }

            if(gamepad2.triangle)
            {
                hardware.clawServo.setPosition(1); //open
            }
            if(gamepad2.circle)
            {
                hardware.clawServo.setPosition(0); //closed
            }
        }
    }
}