
package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private final DcMotorEx outtake;
    private final VoltageSensor battery;

    // Encoder conversion.
    // This must be the encoder ticks per revolution reported by getVelocity().
    public static double TICKS_PER_REV = 28.0;

    // PIDF tunables. These gains are now expressed for RPM error.
    // Converted from the previous ticks/second gains using TICKS_PER_REV = 28.
    public static double kS = 0.08;
    public static double kV = 0.00039 * 28.0 / 60.0;
    public static double kP = 0.01 * 28.0 / 60.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Maximum contribution of the integral term, in motor-power units.
    public static double I_MAX = 0.15;
    public static double NOMINAL_VOLTS = 12.0;

    // RPM equivalents of the previous settings.
    public static double NEAR_RPM = 1500.0 * 60.0 / 28.0;
    public static double TOLERANCE_RPM = 50.0 * 60.0 / 28.0;
    public static double STABLE_SECONDS = 0.15;

    // Original stationary-shot calibration.
    // Kept in ticks/second because MovingShotCalculator uses this curve.
    public static double STATIC_TICKS_QUADRATIC = 0.00180088;
    public static double STATIC_TICKS_LINEAR = 4.14265;
    public static double STATIC_TICKS_CONSTANT = 948.97358;

    // The controller's internal target and measured velocity are RPM.
    private double targetRpm = 0.0;
    private double velocityRpm = 0.0;

    // The calculator can update the target without enabling the motors.
    private boolean activated = false;

    private double integral = 0.0;
    private double lastVelocityRpm = 0.0;
    private double lastTime = 0.0;

    private boolean hasLastVelocity = false;
    private double inTolSince = -1.0;

    public Shooter(HardwareMap hw) {
        outtake = hw.get(DcMotorEx.class, "outtakemotor");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        battery = hw.voltageSensor.iterator().next();
    }

    // --------------------------------------------------
    // Unit conversion
    // --------------------------------------------------

    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    // Compatibility helpers for any existing code using the old names.
    public static double rpmToTicks(double rpm) {
        return rpmToTicksPerSec(rpm);
    }

    public static double ticksToRpm(double ticksPerSec) {
        return ticksPerSecToRpm(ticksPerSec);
    }

    // --------------------------------------------------
    // Target setting
    // --------------------------------------------------

    /**
     * Compatibility method used by MovingShotCalculator.
     *
     * MovingShotCalculator supplies ticks/second.
     * Convert that value to RPM immediately so the shooter
     * controller itself operates entirely in RPM.
     */
    public void setTarget(double ticksPerSec) {
        setTargetRpm(ticksPerSecToRpm(ticksPerSec));
    }

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
            integral = 0.0;
            inTolSince = -1.0;
        }
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getVelocityRpm() {
        return velocityRpm;
    }

    // These existing getters now also return RPM.
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
    // PIDF controller
    // Call this once per OpMode loop.
    // --------------------------------------------------

    public void periodic() {
        double now = System.nanoTime() / 1e9;

        double dt = (lastTime == 0.0)
                ? 0.02
                : Math.max(now - lastTime, 1e-3);

        lastTime = now;

        // DcMotorEx.getVelocity() reports encoder ticks/second.
        // Convert the measured velocity to RPM.
        velocityRpm = ticksPerSecToRpm(outtake.getVelocity());

        if (!activated || targetRpm <= 0.0) {
            setPower(0.0);

            integral = 0.0;
            inTolSince = -1.0;

            lastVelocityRpm = velocityRpm;
            hasLastVelocity = true;

            return;
        }

        double error = targetRpm - velocityRpm;

        // Integral term with anti-windup.
        if (Math.abs(kI) > 1e-9) {
            integral += error * dt;

            double maxIntegral = I_MAX / Math.abs(kI);
            integral = clamp(integral, -maxIntegral, maxIntegral);
        } else {
            integral = 0.0;
        }

        // Derivative on measured velocity avoids a large derivative
        // kick whenever the moving-shot target changes.
        double derivative = 0.0;

        if (hasLastVelocity) {
            derivative = -(velocityRpm - lastVelocityRpm) / dt;
        }

        lastVelocityRpm = velocityRpm;
        hasLastVelocity = true;

        // Feedforward + PID feedback.
        double feedforward = kS + kV * targetRpm;

        double feedback =
                kP * error
                        + kI * integral
                        + kD * derivative;

        // Battery-voltage compensation.
        double volts = Math.max(battery.getVoltage(), 8.0);

        double power =
                (feedforward + feedback)
                        * (NOMINAL_VOLTS / volts);

        setPower(clamp(power, 0.0, 1.0));

        // The measured RPM must remain within tolerance
        // for STABLE_SECONDS before the shooter is ready.
        if (Math.abs(error) < TOLERANCE_RPM) {
            if (inTolSince < 0.0) {
                inTolSince = now;
            }
        } else {
            inTolSince = -1.0;
        }
    }

    public boolean atTarget() {
        return targetRpm > 0.0
                && inTolSince >= 0.0
                && (System.nanoTime() / 1e9 - inTolSince)
                >= STABLE_SECONDS;
    }

    // --------------------------------------------------
    // Motor control
    // --------------------------------------------------

    public void setPower(double power) {
        outtake.setPower(-power);
    }

    // Enables the flywheels without choosing a fixed speed.
    // MovingShotCalculator supplies the target separately.
    public void on() {
        activated = true;
    }

    public void off() {
        activated = false;
        targetRpm = 0.0;

        integral = 0.0;
        inTolSince = -1.0;
        hasLastVelocity = false;

        setPower(0.0);
    }

    // Optional fixed-speed mode for stationary testing.
    public void shootNear() {
        setTargetRpm(NEAR_RPM);
        on();
    }

    // --------------------------------------------------
    // Stationary calibration
    // Kept in ticks/second for MovingShotCalculator.
    // --------------------------------------------------

    public static double ticksForStaticDistance(double distanceInches) {
        return STATIC_TICKS_QUADRATIC * distanceInches * distanceInches
                + STATIC_TICKS_LINEAR * distanceInches
                + STATIC_TICKS_CONSTANT;
    }

    // --------------------------------------------------
    // Ivy commands
    // --------------------------------------------------

    // Fixed-speed command for stationary testing.
    public CommandBuilder shoot() {
        return Commands.instant(this::shootNear);
    }

    // Moving-shot spin-up: enable only.
    // The calculator will continuously supply the target RPM.
    public CommandBuilder spinUp() {
        return Commands.instant(this::on);
    }

    public CommandBuilder stop() {
        return Commands.instant(this::off);
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
