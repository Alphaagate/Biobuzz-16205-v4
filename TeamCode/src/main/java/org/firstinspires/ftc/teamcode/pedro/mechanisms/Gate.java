package org.firstinspires.ftc.teamcode.pedro.mechanisms;


import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Gate {
    private Servo gate;
    public static double down = 0.52;
    public static double up = 0.3;

    public Gate(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void flipDown() {
        gate.setPosition(down);
    }

    public void flipUp() {
        gate.setPosition(up);
    }

    public CommandBuilder up() { return Commands.instant(this::flipUp); }
    public CommandBuilder down() { return Commands.instant(this::flipDown); }

    public void toggle() {
        if (gate.getPosition() == up)
            flipDown();
        else
            flipUp();
    }

    public boolean isDown() {
        return gate.getPosition() == down;
    }

}