package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
/**
 *
 * TODO: * IMPORTANT !!! HOOD SHOULD BE 7 from the top!!!!
 *
 * Flywheel controller. Uses the exact same control law as OwenFlywheelTuner2:
 *
 *     power = P * (target - measured) + kV * target + kS
 *
 * UNITS: "rpm" in this class means the number DcMotorEx.getVelocity() reports. That is the
 * same number OwenFlywheelTuner2 printed, so "1800 close / 2020 far" are used as-is.
 * (getVelocity() is really encoder ticks per second. With a 28-tick encoder,
 * true RPM = this value * 60 / 28.)
 */
public class Shooter {
    private final DcMotorEx outtake;

    // ----- Gains from OwenFlywheelTuner2 (P = 0.005, 0.00041, 0.036) -----
    // In OwenFlywheelTuner2 the values 0.036 and 0.00041 were typed into kV and kS the wrong
    // way around: kV * 1800 would be 64.8 (full power at any target). 0.00041 is the
    // per-velocity-unit term (kV) and 0.036 is the static term (kS), which also matches
    // OwenFlywheel.java (kV = 0.00036, kS = 0.08).
    public static double P = 0.005;
    public static double kV = 0.00041;
    public static double kS = 0.036;

    // "Ready to shoot" window: measured speed must stay within TOLERANCE_RPM of the target
    // for STABLE_SECONDS before atTarget() returns true.
    public static double TOLERANCE_RPM = 50.0;
    public static double STABLE_SECONDS = 0.15;

    private double targetRpm = 0.0;
    private double velocityRpm = 0.0;

    // MovingShotCalculator can update the target while the flywheel is disabled.
    private boolean activated = false;

    private double inTolSince = -1.0;

    public Shooter(HardwareMap hw) {
        outtake = hw.get(DcMotorEx.class, "outtakemotor");

        // Same direction as OwenFlywheelTuner2: positive power and positive velocity.
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // --------------------------------------------------
    // Target setting
    // --------------------------------------------------

    public void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm) || rpm < 0.0) {
            rpm = 0.0;
        }

        // Require the flywheel to become ready again after
        // a meaningful change in the requested speed.
        if (Math.abs(targetRpm - rpm) > TOLERANCE_RPM) {
            inTolSince = -1.0;
        }

        targetRpm = rpm;

        if (targetRpm == 0.0) {
            inTolSince = -1.0;
        }
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getVelocityRpm() {
        return velocityRpm;
    }

    // Existing getter names (used by the autos' telemetry).
    public double getTarget() {
        return targetRpm;
    }

    public double getVelocity() {
        return velocityRpm;
    }

    public double getRpm() {
        return velocityRpm;
    }

    // --------------------------------------------------
    // Controller. Call once per OpMode loop.
    // --------------------------------------------------

    public void periodic() {
        velocityRpm = outtake.getVelocity();

        if (!activated || targetRpm <= 0.0) {
            outtake.setPower(0.0);
            inTolSince = -1.0;
            return;
        }

        double error = targetRpm - velocityRpm;

        double feedback = P * error;
        double feedforward = kV * targetRpm + kS;

        outtake.setPower(clamp(feedback + feedforward, 0.0, 1.0));

        if (Math.abs(error) < TOLERANCE_RPM) {
            if (inTolSince < 0.0) {
                inTolSince = System.nanoTime() / 1e9;
            }
        } else {
            inTolSince = -1.0;
        }
    }

    public boolean atTarget() {
        return targetRpm > 0.0
                && inTolSince >= 0.0
                && (System.nanoTime() / 1e9 - inTolSince) >= STABLE_SECONDS;
    }

    // --------------------------------------------------
    // Motor control
    // --------------------------------------------------

    public void setPower(double power) {
        outtake.setPower(power);
    }

    public void setVelocity(double velocity) {
        outtake.setVelocity(velocity);
    }

    // Enables the flywheel without choosing a speed.
    // MovingShotCalculator supplies the target separately.
    public void on() {
        activated = true;
    }

    public void off() {
        activated = false;
        targetRpm = 0.0;
        inTolSince = -1.0;
        outtake.setPower(0.0);
    }

    public CommandBuilder waitReady() {
        return Commands.waitUntil(this::atTarget);
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    public String getLeftCurrent() {
        return "Left: " + outtake.getCurrent(CurrentUnit.AMPS);
    }
}