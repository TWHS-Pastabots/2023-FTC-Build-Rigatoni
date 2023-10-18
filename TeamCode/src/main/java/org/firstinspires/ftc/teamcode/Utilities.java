package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
public class Utilities
{
    private Hardware hardware;

    public Utilities(Hardware hardware)
    {
        this.hardware = hardware;
    }

    public void wait(int waitTime)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime){}
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
        hardware.launcherReleaseServo.setPosition(1);
        wait(10);
        hardware.launcherTriggerServo.setPosition(1);
        wait(1000);
        hardware.launcherTriggerServo.setPosition(0);
        hardware.launcherReleaseServo.setPosition(0);
    }
}