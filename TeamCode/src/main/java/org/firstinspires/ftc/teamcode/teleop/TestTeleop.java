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

@TeleOp(name="Test")
public class TestTeleop extends OpMode {
    Hardware hardware;
    Utilities utilities;
    double maxVelocity;

    @Override
    public void init() {
        hardware = new Hardware();
        Assert.assertNotNull(hardwareMap);
        hardware.init(hardwareMap);

        utilities = new Utilities(hardware);

        maxVelocity = 0;
        telemetry.addData("Status:: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status:: ", "Started");
        telemetry.update();
        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetry();
        arm();
        launcher();
    }
    public void telemetry()
    {
        telemetry.addData("Max arm velocity", maxVelocity);
        telemetry.addData("Flywheel velocity", hardware.flywheel.getVelocity());
        telemetry.update();
    }
    public void arm()
    {
        hardware.arm.setPower(-gamepad2.right_stick_y);
        double currentVelocity = hardware.arm.getVelocity();
        if(currentVelocity > maxVelocity)
        {
            maxVelocity = currentVelocity;
        }
    }
    public void launcher()
    {
        if(gamepad2.cross)
        {
            hardware.flywheel.setVelocity(400);
        }
        if(gamepad2.circle)
        {
            hardware.flywheel.setPower(1);
        }
    }
}