package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private final DcMotorEx intake;
    public static final double OFF = 0;
    public static final double IDLE = 0.5;
    public static final double INTAKE_POWER = 1;
    public static final double SPITOUT_POWER = -1;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "i");
        set(0);
    }

    public void set(double power) {
        intake.setPower(power);
    }

    public void spinIdle() {
        set(IDLE);
    }

    public CommandBuilder off() {
        return Commands.instant(() -> set(OFF));
    }

    public CommandBuilder intake() {
        return Commands.instant(() -> set(INTAKE_POWER));
    }

    public CommandBuilder spitout() {
        return Commands.instant(() -> set(SPITOUT_POWER));
    }

    public CommandBuilder idle() {
        return Commands.instant(this::spinIdle);
    }


    public String getCurrent() {
        return "Intake Motor: " + intake.getCurrent(CurrentUnit.AMPS);
    }
}
