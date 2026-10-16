package org.firstinspires.ftc.teamcode.pedro.mechanisms;


import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Gate {
    private Servo gate;
    public static double DOWN = 0.52;
    public static double UP = 0.3;

    public Gate(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void flipDown() {
        gate.setPosition(DOWN);
    }

    public void flipUp() {
        gate.setPosition(UP);
    }

    public CommandBuilder up() { return Commands.instant(this::flipUp); }
    public CommandBuilder down() { return Commands.instant(this::flipDown); }

    public void toggle() {
        if (gate.getPosition() == UP)
            flipDown();
        else
            flipUp();
    }

    public boolean isDown() {
        return gate.getPosition() == DOWN;
    }

}