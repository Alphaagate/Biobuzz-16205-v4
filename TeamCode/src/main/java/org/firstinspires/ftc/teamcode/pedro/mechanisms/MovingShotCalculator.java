package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * Calculates a no-drag, velocity-compensated shot in Pedro's field coordinate system.
 *
 * <p>Distances/heights are inches, velocities are inches/second. Flywheel speed is "rpm" in
 * Shooter's units (the number DcMotorEx.getVelocity() reports, same as OwenFlywheelTuner2).
 * Pedro's {@link Follower#velocity()} is world-frame velocity, so no heading rotation is
 * needed before it is subtracted from the desired ball velocity.</p>
 *
 * <p>How the flywheel speed is chosen:
 * <ol>
 *   <li>Standstill speed from distance: a straight line through the two measured points
 *       (CAL_NEAR_DIST_IN -> CAL_NEAR_RPM, CAL_FAR_DIST_IN -> CAL_FAR_RPM).</li>
 *   <li>Moving correction: physics gives the launch speed needed standing still and the launch
 *       speed needed after subtracting the robot's velocity. Their ratio scales the standstill
 *       rpm. Driving toward the goal lowers rpm, driving away raises it.</li>
 * </ol>
 * At standstill the ratio is exactly 1, so the calibration points are reproduced.</p>
 */
public class MovingShotCalculator {
    // ----- Target -----
    // Field X/Y of the target lives in ServoTurret.GOAL_X / GOAL_Y so there is one source of truth.
    public static double TARGET_HEIGHT_IN = 59.0;
    public static double SHOOTER_EXIT_HEIGHT_IN = 13.0;
    //TODO: The above value is a placeholder. Measure the actual height of the shooter exit from the floor.

    // The ball should be travelling downward through the target at this angle.
    public static double TARGET_ENTRY_ANGLE_DEGREES = -45.0;
    public static double GRAVITY_IN_PER_SEC_SQUARED = 386.09;

    // ----- Standstill calibration (goal at (60, 60)) -----
    // Robot at (39, 9): 1800 scores.   Robot at (9, 9): 2020 scores.
    // Distances are robot-center-to-goal; the 1 in turret offset changes the result by < 10 rpm.
    public static double CAL_NEAR_DIST_IN = Math.hypot(60.0 - 39.0, 60.0 - 9.0); // ~55.15 in
    public static double CAL_NEAR_RPM = 1800.0;
    public static double CAL_FAR_DIST_IN = Math.hypot(60.0 - 9.0, 60.0 - 9.0);   // ~72.12 in
    public static double CAL_FAR_RPM = 2020.0;

    /**
     * How strongly rpm follows the physics speed ratio while moving.
     * 1.0 = rpm scales in proportion to required launch speed.
     * Tune while driving straight toward / away from the goal:
     *   - short when driving away, long when driving toward -> rpm is under-corrected, raise it
     *   - long when driving away, short when driving toward -> rpm is over-corrected, lower it
     * 0.0 turns the rpm correction off (the turret lead still works).
     */
    public static double SPEED_COMP_GAIN = 1.0;

    // Reject degenerate or physically impossible solutions instead of commanding a bad shot.
    public static double MIN_DISTANCE_IN = 6.0;
    public static double MIN_RPM = 0.0;
    // With kV = 0.00041 and kS = 0.036 full power is about 2350, so don't ask for more than this.
    public static double MAX_RPM = 2300.0;

    /** Set false only when a fixed/manual shooter target is wanted for a test. */
    public static boolean APPLY_SHOOTER_SPEED = true;

    private final Shooter shooter;
    private final ServoTurret turret;
    private ShotSolution solution = ShotSolution.invalid();

    public MovingShotCalculator(Shooter shooter, ServoTurret turret) {
        this.shooter = shooter;
        this.turret = turret;
    }

    /**
     * Calculates and applies the latest solution. Call this after follower.update() every loop.
     */
    public void update(Follower follower) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        solution = calculate(pose, velocity);

        if (!solution.valid) {
            return;
        }

        turret.aimAtFieldAngle(pose, solution.turretFieldAngle, solution.distanceToGoal);
        if (APPLY_SHOOTER_SPEED) {
            shooter.setTargetRpm(solution.flywheelRpm);
        }
    }

    /** Standstill flywheel speed for a given distance to the goal (straight line through the 2 calibration points). */
    public static double rpmForStaticDistance(double distanceInches) {
        double slope = (CAL_FAR_RPM - CAL_NEAR_RPM) / (CAL_FAR_DIST_IN - CAL_NEAR_DIST_IN);
        return CAL_NEAR_RPM + slope * (distanceInches - CAL_NEAR_DIST_IN);
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

        double flightTime = distance / stationaryHorizontalSpeed;

        // Standstill rpm for this spot on the field, then scaled by how much faster/slower the
        // ball has to leave the shooter because the robot is moving.
        double speedRatio = compensatedLaunchSpeed / stationaryLaunchSpeed;
        double flywheelRpm = rpmForStaticDistance(distance)
                * (1.0 + SPEED_COMP_GAIN * (speedRatio - 1.0));
        flywheelRpm = clamp(flywheelRpm, MIN_RPM, MAX_RPM);

        if (!isFinite(flightTime) || !isFinite(flywheelRpm)) {
            return ShotSolution.invalid();
        }

        return new ShotSolution(true, distance, flightTime, stationaryLaunchSpeed,
                compensatedLaunchSpeed, turretFieldAngle,
                flywheelRpm, turretVelocityX, turretVelocityY);
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
        public final double flywheelRpm;
        public final double turretVelocityX;
        public final double turretVelocityY;

        private ShotSolution(boolean valid, double distanceToGoal, double flightTime,
                             double stationaryLaunchSpeed, double compensatedLaunchSpeed,
                             double turretFieldAngle,
                             double flywheelRpm, double turretVelocityX,
                             double turretVelocityY) {
            this.valid = valid;
            this.distanceToGoal = distanceToGoal;
            this.flightTime = flightTime;
            this.stationaryLaunchSpeed = stationaryLaunchSpeed;
            this.compensatedLaunchSpeed = compensatedLaunchSpeed;
            this.turretFieldAngle = turretFieldAngle;
            this.flywheelRpm = flywheelRpm;
            this.turretVelocityX = turretVelocityX;
            this.turretVelocityY = turretVelocityY;
        }

        private static ShotSolution invalid() {
            return new ShotSolution(false, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }
}