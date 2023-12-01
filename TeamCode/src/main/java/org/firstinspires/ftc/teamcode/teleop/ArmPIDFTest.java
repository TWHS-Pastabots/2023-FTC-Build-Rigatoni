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

    double kFMultiplier;

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
        calculateKFMultiplier();
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
        telemetry.addData("kF multiplier", kFMultiplier);
        telemetry.update();
    }

    public void calculateKFMultiplier() {
        // Feedforwards based on force of gravity
        kFMultiplier = Math.cos((hardware.arm.getCurrentPosition() - 5) * Math.PI / 144);
    }

    public void setPIDFCoefficients() {
        if(buttonTime.time() > 250)
        {
            // kP
            if(gamepad1.dpad_up)
            {
                kP += incrementMultiplier;
                buttonTime.reset();
            }
            if(gamepad1.dpad_down)
            {
                kP -= incrementMultiplier;
                buttonTime.reset();
            }
            // kI
            if(gamepad1.dpad_left)
            {
                kI -= incrementMultiplier;
                buttonTime.reset();
            }
            if(gamepad1.dpad_right)
            {
                kI += incrementMultiplier;
                buttonTime.reset();
            }
            // kD
            if(gamepad1.triangle)
            {
                kD += incrementMultiplier;
                buttonTime.reset();
            }
            if(gamepad1.cross)
            {
                kD -= incrementMultiplier;
                buttonTime.reset();
            }
            //kF
            if(gamepad1.square)
            {
                kF -= incrementMultiplier;
                buttonTime.reset();
            }
            if(gamepad1.circle)
            {
                kF += incrementMultiplier;
                buttonTime.reset();
            }

            // Multiplier
            if(gamepad1.right_bumper)
            {
                incrementMultiplier *= 10;
                buttonTime.reset();
            }
            if(gamepad1.left_bumper)
            {
                incrementMultiplier /= 10;
                buttonTime.reset();
            }
        }
    }

    private void arm()
    {
        if(gamepad1.right_trigger > 0.1)
        {
            armPosition = 125;
        }
        else if(gamepad1.left_trigger > 0.1)
        {
            armPosition = 190;
        }
        else if(gamepad1.share)
        {
            armPosition = 0;
        }

        // Arm position
        hardware.arm.setTargetPosition(armPosition);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.25);
        hardware.arm.setVelocityPIDFCoefficients(kP, kI, kD, kF * kFMultiplier);

    }
}