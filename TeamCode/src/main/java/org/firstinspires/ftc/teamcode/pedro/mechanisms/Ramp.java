package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ramp {

    private Servo ramp;
    public static final double DOWN = 0;
    public static final double UP = 1;

    public Ramp(HardwareMap hardwareMap) {
        ramp = hardwareMap.get(Servo.class, "ramp");
    }

    public void rampDown() {
        ramp.setPosition(DOWN);
    }

    public void rampUp() {
        ramp.setPosition(UP);
    }

    public CommandBuilder up() { return Commands.instant(this::rampUp); }
    public CommandBuilder down() { return Commands.instant(this::rampDown); }

    public void toggle() {
        if (ramp.getPosition() == UP)
            rampDown();
        else
            rampUp();
    }

    public boolean isDown() {
        return ramp.getPosition() == DOWN;
    }

}

