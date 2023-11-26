package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="ArmPIDFTest")
public class ArmPIDFTest extends OpMode
{
    Hardware hardware;
    Utilities utilities;

    int armPosition;

    double kP;
    double kI;
    double kD;
    double kF;

    double incrementMultiplier;
    ElapsedTime buttonTime;

    PIDFCoefficients pidfCoefficients;

    @Override
    public void init() {
        hardware = new Hardware();
        Assert.assertNotNull(hardwareMap);
        hardware.init(hardwareMap);

        utilities = new Utilities(hardware);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        incrementMultiplier = 1;
        kP = 0;
        kI = 0;
        kD = 0;
        kF = 0;
        pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);

        armPosition = 0;
        hardware.arm.setTargetPosition(armPosition);
        hardware.arm.setPower(0.5);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status:: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status:: ", "Started");
        telemetry.update();

    }

    @Override
    public void loop() {
        setPIDFCoefficients();
        arm();
        telemetry();

    }

    public void telemetry() {
        telemetry.addData("Arm position", armPosition);
        telemetry.addData("Arm position actual", hardware.arm.getCurrentPosition());
        telemetry.addData("Increment multiplier", incrementMultiplier);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }

    public void setPIDFCoefficients() {
        if(buttonTime.time() > 250)
        {
            // kP
            if(gamepad1.dpad_up)
            {
                kP += incrementMultiplier;
            }
            if(gamepad1.dpad_down)
            {
                kP -= incrementMultiplier;
            }
            // kI
            if(gamepad1.dpad_left)
            {
                kI -= incrementMultiplier;
            }
            if(gamepad1.dpad_right)
            {
                kI += incrementMultiplier;
            }
            // kD
            if(gamepad1.triangle)
            {
                kD += incrementMultiplier;
            }
            if(gamepad1.cross)
            {
                kD -= incrementMultiplier;
            }
            //kF
            if(gamepad1.square)
            {
                kF -= incrementMultiplier;
            }
            if(gamepad1.circle)
            {
                kF += incrementMultiplier;
            }

            // Multiplier
            if(gamepad1.right_bumper)
            {
                incrementMultiplier *= 10;
            }
            if(gamepad1.left_bumper)
            {
                incrementMultiplier /= 10;
            }

            pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
        }
    }

    private void arm()
    {
        if(gamepad2.right_bumper)
        {
            armPosition = 125;
        }
        else if(gamepad2.left_bumper)
        {
            armPosition = 190;
        }
        else if(gamepad2.circle)
        {
            armPosition = 0;
        }

        // Arm position
        hardware.arm.setTargetPosition(armPosition);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.25);
        hardware.arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);

    }
}