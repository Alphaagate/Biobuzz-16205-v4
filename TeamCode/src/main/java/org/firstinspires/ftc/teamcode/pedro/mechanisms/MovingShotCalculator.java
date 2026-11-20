package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * Calculates a no-drag, velocity-compensated shot in Pedro's field coordinate system.
 *
 * <p>All distance, height, and velocity values are inches, inches/second, and
 * inches/second^2. Pedro's {@link Follower#velocity()} is world-frame velocity, so no
 * heading rotation is needed before it is subtracted from the desired ball velocity.</p>
 */
public class MovingShotCalculator {
    // Field target. These are shared with ServoTurret so there is exactly one X/Y target.
    // Set these to the center of the CELL opening that is currently intended to score.
    public static double TARGET_HEIGHT_IN = 59.5;
    public static double SHOOTER_EXIT_HEIGHT_IN = 13.0;
    //TODO: The above value is a placeholder. Measure the actual height of the shooter exit from the floor.

    // The ball should be travelling downward through the target at this angle.
    public static double TARGET_ENTRY_ANGLE_DEGREES = -45.0;
    public static double GRAVITY_IN_PER_SEC_SQUARED = 386.09;

    // Reject degenerate or physically impossible solutions instead of commanding a bad shot.
    public static double MIN_DISTANCE_IN = 6.0;
    public static double MIN_FLYWHEEL_TICKS_PER_SEC = 0.0;
    public static double MAX_FLYWHEEL_TICKS_PER_SEC = 3000.0;

    /** Set false only when a fixed/manual shooter target is wanted for a test. */
    public static boolean APPLY_SHOOTER_SPEED = true;

    private final Shooter shooter;
    private final ServoTurret turret;
    private final Hood hood;
    private ShotSolution solution = ShotSolution.invalid();

    public MovingShotCalculator(Shooter shooter, ServoTurret turret, Hood hood) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
    }

    /**
     * Calculates and applies the latest solution. Call this after follower.update() every loop.
     */
    public void update(Follower follower) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        solution = calculate(pose, velocity);

        if (!solution.valid) {
            // Keep the legacy, stationary aim available when a configuration is invalid.
//            turret.aimAtFieldPoint(pose, ServoTurret.GOAL_X, ServoTurret.GOAL_Y);
            return;
        }

        turret.aimAtFieldAngle(pose, solution.turretFieldAngle, solution.distanceToGoal);
        if (APPLY_SHOOTER_SPEED) {
            shooter.setTarget(solution.flywheelTicksPerSec);
        }
    }

    /**
     * Solves the shot without writing hardware. Kept public so it can be plotted or unit-tested.
     *
     * <p>For the stationary trajectory, launch pitch alpha is derived from target height,
     * horizontal range d, and target-entry pitch beta:
     * tan(alpha) = 2 * height / d - tan(beta).
     * The world-frame ball velocity is then (vHorizontal * unitToGoal, vVertical).
     * Subtracting the turret's world velocity yields the velocity the shooter must impart.
     * This simultaneously produces the flywheel-speed correction and turret lead.</p>
     */
    public ShotSolution calculate(Pose robotPose, Velocity robotVelocity) {
        double turretX = turret.getTurretX(robotPose);
        double turretY = turret.getTurretY(robotPose);
        double deltaX = ServoTurret.GOAL_X - turretX;
        double deltaY = ServoTurret.GOAL_Y - turretY;
        double distance = Math.hypot(deltaX, deltaY);

        if (distance < MIN_DISTANCE_IN || !isFinite(distance)) {
            return ShotSolution.invalid();
        }

        double height = TARGET_HEIGHT_IN - SHOOTER_EXIT_HEIGHT_IN;
        double entryAngle = Math.toRadians(TARGET_ENTRY_ANGLE_DEGREES);
        double launchAngle = Math.atan(2.0 * height / distance - Math.tan(entryAngle));
        double cosLaunch = Math.cos(launchAngle);
        double denominator = cosLaunch * cosLaunch
                * (Math.tan(launchAngle) - Math.tan(entryAngle));

        if (denominator <= 1e-6 || !isFinite(denominator)) {
            return ShotSolution.invalid();
        }

        double stationaryLaunchSpeed = Math.sqrt(
                GRAVITY_IN_PER_SEC_SQUARED * distance / denominator);
        double stationaryHorizontalSpeed = stationaryLaunchSpeed * cosLaunch;
        double stationaryVerticalSpeed = stationaryLaunchSpeed * Math.sin(launchAngle);

        // The velocity of the turret pivot includes the small tangential component created
        // when the chassis rotates around its center.
        double offsetX = turretX - robotPose.x();
        double offsetY = turretY - robotPose.y();
        double turretVelocityX = robotVelocity.vx - robotVelocity.omega * offsetY;
        double turretVelocityY = robotVelocity.vy + robotVelocity.omega * offsetX;

        double unitX = deltaX / distance;
        double unitY = deltaY / distance;
        double requiredLaunchX = stationaryHorizontalSpeed * unitX - turretVelocityX;
        double requiredLaunchY = stationaryHorizontalSpeed * unitY - turretVelocityY;
        double compensatedHorizontalSpeed = Math.hypot(requiredLaunchX, requiredLaunchY);
        double compensatedLaunchSpeed = Math.hypot(
                compensatedHorizontalSpeed, stationaryVerticalSpeed);

        if (compensatedHorizontalSpeed < 1e-6 || !isFinite(compensatedLaunchSpeed)) {
            return ShotSolution.invalid();
        }

        double turretFieldAngle = Math.atan2(requiredLaunchY, requiredLaunchX);
        double compensatedHoodAngle = Math.atan2(
                stationaryVerticalSpeed, compensatedHorizontalSpeed);
        double flightTime = distance / stationaryHorizontalSpeed;

        // The existing team calibration maps a stationary distance to motor ticks/sec.
        // Ball exit speed is approximately proportional to flywheel speed over the tuned range,
        // so scale that empirical target by the exact velocity-compensation ratio.
        double stationaryFlywheelTicks = Shooter.ticksForStaticDistance(distance);
        double flywheelTicks = stationaryFlywheelTicks
                * compensatedLaunchSpeed / stationaryLaunchSpeed;
        flywheelTicks = clamp(flywheelTicks,
                MIN_FLYWHEEL_TICKS_PER_SEC, MAX_FLYWHEEL_TICKS_PER_SEC);

        if (!isFinite(flightTime) || !isFinite(flywheelTicks)) {
            return ShotSolution.invalid();
        }

        return new ShotSolution(true, distance, flightTime, stationaryLaunchSpeed,
                compensatedLaunchSpeed, turretFieldAngle, compensatedHoodAngle,
                flywheelTicks, turretVelocityX, turretVelocityY);
    }

    public ShotSolution getSolution() {
        return solution;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public static class ShotSolution {
        public final boolean valid;
        public final double distanceToGoal;
        public final double flightTime;
        public final double stationaryLaunchSpeed;
        public final double compensatedLaunchSpeed;
        public final double turretFieldAngle;
        public final double hoodAngle;
        public final double flywheelTicksPerSec;
        public final double turretVelocityX;
        public final double turretVelocityY;

        private ShotSolution(boolean valid, double distanceToGoal, double flightTime,
                             double stationaryLaunchSpeed, double compensatedLaunchSpeed,
                             double turretFieldAngle, double hoodAngle,
                             double flywheelTicksPerSec, double turretVelocityX,
                             double turretVelocityY) {
            this.valid = valid;
            this.distanceToGoal = distanceToGoal;
            this.flightTime = flightTime;
            this.stationaryLaunchSpeed = stationaryLaunchSpeed;
            this.compensatedLaunchSpeed = compensatedLaunchSpeed;
            this.turretFieldAngle = turretFieldAngle;
            this.hoodAngle = hoodAngle;
            this.flywheelTicksPerSec = flywheelTicksPerSec;
            this.turretVelocityX = turretVelocityX;
            this.turretVelocityY = turretVelocityY;
        }

        private static ShotSolution invalid() {
            return new ShotSolution(false, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
}
