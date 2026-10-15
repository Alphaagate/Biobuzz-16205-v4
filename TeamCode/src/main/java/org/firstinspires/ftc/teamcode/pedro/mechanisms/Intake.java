package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private final DcMotorEx intake;
    public static double off = 0;
    public static double idle = 0.5;
    public static double intakePower = 1;
    public static double spitoutPower = -1;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "i");
        set(0);
    }

    public void set(double power) {
        intake.setPower(power);
    }

    public void spinIdle() {
        set(idle);
    }

    public CommandBuilder off() {
        return Commands.instant(() -> set(off));
    }

    public CommandBuilder intake() {
        return Commands.instant(() -> set(intakePower));
    }

    public CommandBuilder spitout() {
        return Commands.instant(() -> set(spitoutPower));
    }

    public CommandBuilder idle() {
        return Commands.instant(this::spinIdle);
    }


    public String getCurrent() {
        return "Intake Motor: " + intake.getCurrent(CurrentUnit.AMPS);
    }
}
