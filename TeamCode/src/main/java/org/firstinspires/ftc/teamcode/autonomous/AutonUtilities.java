package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
public class AutonUtilities
{
    private Hardware hardware;

    public AutonUtilities(Hardware hardware)
    {
        this.hardware = hardware;
    }

    public void wait(int waitTime)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime){}
    }

    public void shoot()
    {
        hardware.launcherTriggerServo.setPosition(1);
        wait(500);
        hardware.launcherTriggerServo.setPosition(.2);
    }

    public void deployIntake()
    {
        hardware.intakeDeployServo1.setPosition(0.10);
        hardware.intakeDeployServo2.setPosition(0.60);
    }

    public void retractIntake()
    {
        hardware.intakeDeployServo1.setPosition(0.50);
        hardware.intakeDeployServo2.setPosition(0.20);

    }

    public void intake(int intakeTime)
    {
        hardware.intake.setPower(1.0);
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (time.time() < intakeTime){}
    }
    public void intake(boolean on)
    {
        if(on)
            hardware.intake.setPower(1);
        else
            hardware.intake.setPower(0);
    }

    public void aimLauncher(double position)
    {
        hardware.launcherAimServo.setPosition(position);
    }
    public void flywheelVelocity(double velocity)
    {
        hardware.flywheel.setVelocity(velocity);
    }
    public void flywheelVelocity(double velocity, int duration)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(time.time() < duration)
        {
            hardware.flywheel.setVelocity(velocity);
        }
    }
    public void flywheelOff() {hardware.flywheel.setVelocity(0);}
}