package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="Checkpoint4")
public class ArmCustomPIDF extends OpMode {
    Hardware hardware;
    Utilities utilities;


    int armTargetPosition;

    PIDFCoefficients pidfCoefficients;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    double integralSum;
    double lastError;

    double incrementMultiplier;

    ElapsedTime armTime;
    ElapsedTime buttonTime;


    @Override
    public void init() {
        hardware = new Hardware();
        Assert.assertNotNull(hardwareMap);
        hardware.init(hardwareMap);

        utilities = new Utilities(hardware);

        incrementMultiplier = 1.0;

        armTargetPosition = 0;
        hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status:: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
        telemetry.addData("Arm position:: ", armTargetPosition);
        telemetry.addData("Arm position actual:: ", hardware.arm.getCurrentPosition());
        telemetry.update();
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
            if(gamepad1.right_trigger > 0.1)
            {
                incrementMultiplier *= 10;
                buttonTime.reset();
            }
            if(gamepad1.left_trigger > 0.1)
            {
                incrementMultiplier /= 10;
                buttonTime.reset();
            }
        }
    }

    private void arm()
    {
        if(gamepad2.right_bumper)
        {
            armTargetPosition = 125; // Junction
        }
        else if(gamepad2.left_bumper)
        {
            armTargetPosition = 190; // Ground
        }
        else if(gamepad2.share)
        {
            armTargetPosition = 0; // Retracted
        }

        // Arm power
        hardware.arm.setPower(pidfCalculation(armTargetPosition, hardware.arm.getCurrentPosition()));
    }

    public double pidfCalculation(int reference, int state)
    {
        double error = reference - state;
        integralSum += error * armTime.seconds();
        double derivative = (error - lastError) / armTime.seconds();
        lastError = error;
        armTime.reset();
        return Math.min((error * kP) + (derivative * kD) + (integralSum + kI) + (calculateKFMultiplier() * kF), 1.0);
    }

    public double calculateKFMultiplier() {
        // Feedforwards based on force of gravity
        return Math.cos((hardware.arm.getCurrentPosition() - 5) * Math.PI / 144);
    }
}