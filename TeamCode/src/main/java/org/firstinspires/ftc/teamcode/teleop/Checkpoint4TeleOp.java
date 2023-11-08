package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    boolean controlOverride;
    boolean intakeOn;
    boolean clawOpen;

    final double INTAKE_SPEED = 1;
    final double ARM_SPEED = 1;
    final double LAUNCHER_AIM_LOW_BOUND = 0;
    final double LAUNCHER_AIM_HIGH_BOUND = 1;

    double launcherAimServoPosition;

    final double INTAKE_DEPLOY_MAX_POSITION = 1;
    final double INTAKE_DEPLOY_MIN_POSITION = 0;
    double intakeDeployServoPosition;

    final double FLYWHEEL_FAST_CAP = 1;
    final double FLYWHEEL_SLOW_CAP = .4;
    double flywheelSpeed;

    int armPosition;

    // ElapsedTime
    ElapsedTime flywheelTime;
    ElapsedTime launcherAimTime;
    ElapsedTime sinceStartTime;
    ElapsedTime intakeTime;
    ElapsedTime intakeDeployTime;
    ElapsedTime clawTime;

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
        intakeOn = false;
        clawOpen = false;
        launcherAimServoPosition = 0;
        intakeDeployServoPosition = 0;
        flywheelSpeed = 1.0;

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
        intakeDeployTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
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
        } else if (gamepad1.circle) {
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
        // Intake power
        if(gamepad1.square && intakeTime.time() >= 250)
        {
            intakeOn = !intakeOn;
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

        // Intake deployment
        if(gamepad2.right_stick_y >= 0.1 && intakeDeployTime.time() >= 50)
        {
            intakeDeployTime.reset();
            if(gamepad2.right_stick_y > 0)
            {
                intakeDeployServoPosition = Math.min(intakeDeployServoPosition + 0.05, INTAKE_DEPLOY_MAX_POSITION);
            }
            else
            {
                intakeDeployServoPosition = Math.max(intakeDeployServoPosition - 0.05, INTAKE_DEPLOY_MIN_POSITION);
            }
        }
        hardware.intakeDeployServo.setPosition(intakeDeployServoPosition);


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
        // Prevent accidental arm trigger
        if(gamepad2.options)
        {
            controlOverride = true;
        }
        // If endgame or overridden allow arm use
        if(sinceStartTime.time() > 90000 || controlOverride)
        {
//            if(Math.abs(gamepad2.left_stick_y) > 0.10) //Setting deadzone for arm breaking
//            {
//                if(!(hardware.arm.getMode() == DcMotor.RunMode.RUN_USING_ENCODER))
//                {
//                    hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
                hardware.arm.setPower(-gamepad2.left_stick_y * ARM_SPEED);
//                armPosition = hardware.arm.getCurrentPosition();
//            }
//            else // PID control for arm breaking
//            {
//                hardware.arm.setTargetPosition(armPosition);
//                hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

            // Claw
            if(gamepad2.triangle && clawTime.time() >= 250)
            {
                clawOpen = !clawOpen;
                utilities.clawControl(clawOpen);
            }
        }
    }
}