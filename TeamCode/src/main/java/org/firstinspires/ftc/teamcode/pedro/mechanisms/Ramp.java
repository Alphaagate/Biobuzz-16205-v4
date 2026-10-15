package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ramp {

    private Servo ramp;
    public static double down = 0.52;
    public static double up = 0.3;

    public Ramp(HardwareMap hardwareMap) {
        ramp = hardwareMap.get(Servo.class, "ramp");
    }

    public void rampDown() {
        ramp.setPosition(down);
    }

    public void rampUp() {
        ramp.setPosition(up);
    }

    public CommandBuilder up() { return Commands.instant(this::rampUp); }
    public CommandBuilder down() { return Commands.instant(this::rampDown); }

    public void toggle() {
        if (ramp.getPosition() == up)
            rampDown();
        else
            rampUp();
    }

    public boolean isDown() {
        return ramp.getPosition() == down;
    }

}

