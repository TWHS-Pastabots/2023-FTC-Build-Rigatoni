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
            hardware.clawServo.setPosition(1); //open
        else
            hardware.clawServo.setPosition(0); //closed
    }

    public void shoot()
    {
        hardware.launcherTriggerServo.setPosition(1);
        wait(500);
        hardware.launcherTriggerServo.setPosition(.2);
    }
}