package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private final DcMotorEx left, right;
    private final VoltageSensor battery;

    // ---- Tunables (public static so FTC Dashboard can edit them) ----
    public static double kS = 0.08;        // power to just barely move
    public static double kV = 0.00039;     // power per (tick/s)
    public static double kP = 0.01;
    public static double kI = 0.0;         // start at 0
    public static double kD = 0.0;         // usually leave 0 for flywheels
    public static double I_MAX = 0.15;     // max power the I term may contribute
    public static double NOMINAL_VOLTS = 12.0;

    public static double TICKS_PER_REV = 28;   // CHECK YOUR MOTOR
    public static double NEAR = 1500;          // ticks/s (see note above)
    public static double TOLERANCE = 50;       // ticks/s
    public static double STABLE_SECONDS = 0.15;

    private double target = 0;
    private boolean activated = true;

    private double integral = 0, lastError = 0, lastTime = 0;
    private double inTolSince = -1;
    private double velocity = 0;               // cached once per loop

    public Shooter(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "shooterleft");
        right = hw.get(DcMotorEx.class, "shooterright");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        battery = hw.voltageSensor.iterator().next();
    }

    // ---------- unit helpers ----------
    public static double rpmToTicks(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    public static double ticksToRpm(double t)   { return t * 60.0 / TICKS_PER_REV; }

    // ---------- control ----------
    public void setTarget(double ticksPerSec) { target = ticksPerSec; }
    public void setTargetRpm(double rpm)      { target = rpmToTicks(rpm); }
    public double getTarget()   { return target; }
    public double getVelocity() { return velocity; }
    public double getRpm()      { return ticksToRpm(velocity); }

    public void periodic() {
        double now = System.nanoTime() / 1e9;
        double dt = (lastTime == 0) ? 0.02 : Math.max(now - lastTime, 1e-3);
        lastTime = now;

        velocity = right.getVelocity();            // ONE hardware read per loop

        if (!activated || target <= 0) {
            setPower(0);
            integral = 0; lastError = 0; inTolSince = -1;
            return;
        }

        double error = target - velocity;

        // Integral with anti-windup (clamped in power units)
        integral += error * dt;
        if (kI != 0) integral = clamp(integral, -I_MAX / kI, I_MAX / kI);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double feedforward = kS + kV * target;
        double feedback = kP * error + kI * integral + kD * derivative;

        // Compensate for battery sag
        double volts = Math.max(battery.getVoltage(), 8.0);
        double power = (feedforward + feedback) * (NOMINAL_VOLTS / volts);

        setPower(clamp(power, 0, 1));              // never spin backwards

        // "at speed" must hold for a short time, not just one lucky sample
        if (Math.abs(error) < TOLERANCE) {
            if (inTolSince < 0) inTolSince = now;
        } else {
            inTolSince = -1;
        }
    }

    public boolean atTarget() {
        return target > 0 && inTolSince >= 0
                && (System.nanoTime() / 1e9 - inTolSince) >= STABLE_SECONDS;
    }

    public void setPower(double p) { left.setPower(-p); right.setPower(-p); }
    public void on()  { activated = true; }
    public void off() { activated = false; target = 0; setPower(0); }

    public void shootNear() { setTarget(NEAR); on(); }

    // ---------- Ivy commands ----------
    public CommandBuilder shoot()     { return Commands.instant(this::shootNear); }
    public CommandBuilder spinUp()    { return Commands.instant(this::shootNear); }
    public CommandBuilder stop()      { return Commands.instant(this::off); }
    public CommandBuilder waitReady() { return Commands.waitUntil(this::atTarget); }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public String getLeftCurrent()  { return "Left: "  + left.getCurrent(CurrentUnit.AMPS); }
    public String getRightCurrent() { return "Right: " + right.getCurrent(CurrentUnit.AMPS); }
}