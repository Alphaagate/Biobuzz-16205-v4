package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private final DcMotorEx intake, transfer;
    public static final double OFF = 0;
    public static final double IDLE = 0.5;
    public static final double INTAKE_POWER = 1;
    private static final double TRANSFER_POWER = 1;
    public static final double SPITOUT_POWER = -1;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        set(0, 0);
    }

    public void set(double intakePower, double transferPower) {
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }

    public void spinIdle() {
        set(IDLE, IDLE);
    }

    public CommandBuilder off() {
        return Commands.instant(() -> set(OFF, OFF));
    }

    public CommandBuilder intake() {
        return Commands.instant(() -> set(INTAKE_POWER, TRANSFER_POWER));
    }

    public CommandBuilder spitout() {
        return Commands.instant(() -> set(SPITOUT_POWER, SPITOUT_POWER));
    }

    public CommandBuilder idle() {
        return Commands.instant(this::spinIdle);
    }


    public String getCurrent() {
        return "Intake Motor: " + intake.getCurrent(CurrentUnit.AMPS);
    }
}
