package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    private Servo gate;
    public static final double CLOSED = 1;
    public static final double OPEN = 0;

    public Gate(HardwareMap hardwareMap) {
        gate = hardwareMap.get(Servo.class, "gate");
    }   

    public void gateClosed() {
        gate.setPosition(CLOSED);
    }

    public void gateOpen() {
        gate.setPosition(OPEN);
    }

    public CommandBuilder open() { return Commands.instant(this::gateOpen); }
    public CommandBuilder closed() { return Commands.instant(this::gateClosed); }

    public void toggle() {
        if (gate.getPosition() == OPEN)
            gateClosed();
        else
            gateOpen();
    }

    public boolean isDown() {
        return gate.getPosition() == CLOSED;
    }

}

