package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    final double FAST_SPEED = 1.0;
    final double MID_SPEED = .5;
    final double SLOW_SPEED = .25;

    double speedConstant;
    boolean fieldOriented;

    ElapsedTime sinceStartTime;

    // Field oriented
//    Orientation angles = new Orientation();                 // CHECK IN CASE OF FAILURE
//    double initYaw;
//    double adjustedYaw;

    @Override
    public void init() {
        hardware = new Hardware();
        Assert.assertNotNull(hardwareMap);
        hardware.init(hardwareMap);

        speedConstant = FAST_SPEED;
        // fieldOriented = true;

        // Setup field oriented
        // angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // initYaw = angles.firstAngle;

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
        drive();
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
//        if (gamepad1.options) {
//            fieldOriented = true;
//        } else if (gamepad1.share) {
//            fieldOriented = false;
//        }


        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        // Field oriented
////        if (fieldOriented) {
////        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////
////        adjustedYaw = angles.firstAngle-initYaw;
////
////        double zerodYaw = -initYaw+angles.firstAngle;
////
////        double theta = Math.atan2(y, x) * 180/Math.PI; // aka angle
////
////        double realTheta;
////
////        realTheta = (360 - zerodYaw) + theta;
////
////        double power = Math.hypot(x, y);
////
////        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
////        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
////        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));
//
//        leftFrontPower = (power * cos / maxSinCos + turn);
//        rightFrontPower = (power * sin / maxSinCos - turn);
//        leftRearPower = (power * sin / maxSinCos + turn);
//        rightRearPower = (power * cos / maxSinCos - turn);
//        }
//        else {
        leftFrontPower = y + x + turn;
        leftRearPower = y - x + turn;
        rightFrontPower = y - x - turn;
        rightRearPower = y + x - turn;
//        }

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

        hardware.rightRear.setPower(rightRearPower);
        hardware.rightFront.setPower(rightFrontPower);
        hardware.leftFront.setPower(leftFrontPower);
        hardware.leftRear.setPower(leftRearPower);

    }
}