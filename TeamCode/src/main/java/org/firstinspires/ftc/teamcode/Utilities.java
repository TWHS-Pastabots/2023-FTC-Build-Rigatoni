package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
public class Utilities
{
    private Hardware hardware;

    public Utilities(Hardware hardware)
    {
        this.hardware = hardware;
    }

    public void clawControl(boolean shouldOpen)
    {
        if(shouldOpen)
            hardware.clawServo.setPosition(1.0);
        else
            hardware.clawServo.setPosition(0.0);
    }

    public void shoot()
    {
        hardware.launcherReleaseServo.setPosition(1.0);
        hardware.launcherTriggerServo.setPosition(1.0);
    }
}