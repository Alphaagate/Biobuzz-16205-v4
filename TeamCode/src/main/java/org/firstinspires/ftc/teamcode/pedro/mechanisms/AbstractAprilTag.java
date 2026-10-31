package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public abstract class AbstractAprilTag extends LinearOpMode {

    protected VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagClusterDetection targetCluster;   // best cluster this loop, or null

    /** "R" for red alliance, "B" for blue alliance. */
    protected abstract String getAllianceLetter();

    protected void initAprilTag() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }

    protected void detectAprilTag() {
        targetCluster = null;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("# detections (clusters)", detections.size());

        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

                boolean myAlliance = cluster.metadata.name.startsWith(getAllianceLetter());
                boolean rightSideUp = Math.abs(cluster.ftcPose.roll) < 90;   // scorable cell

                telemetry.addLine(String.format("Cluster: %s | %d%% tags | roll %.0f | %s%s",
                        cluster.metadata.name,
                        cluster.percentClusterFound,
                        cluster.ftcPose.roll,
                        myAlliance ? "MY ALLIANCE" : "other alliance",
                        rightSideUp ? " | SCORABLE" : " | upside-down"));

                if (myAlliance && rightSideUp) {
                    // Prefer the cluster with the most tags visible
                    if (targetCluster == null
                            || cluster.percentClusterFound > targetCluster.percentClusterFound) {
                        targetCluster = cluster;
                    }
                }

            } else if (detection instanceof AprilTagSingleDetection) {
                // BIOBUZZ has no single tags, so this should only be stray/test tags
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                telemetry.addData("Unexpected single tag ID", single.id);
            }
        }

        if (targetCluster == null) {
            telemetry.addData("Target", "none (no scorable cluster for my alliance)");
        } else {
            telemetry.addLine(String.format("TARGET %s  range %.1f in  bearing %.1f deg  elevation %.1f deg",
                    targetCluster.metadata.name,
                    targetCluster.ftcPose.range,
                    targetCluster.ftcPose.bearing,
                    targetCluster.ftcPose.elevation));
        }
    }

    protected void stopVisionPortal() {
        if (visionPortal != null) visionPortal.close();
    }

    protected AprilTagClusterDetection getTargetCluster() { return targetCluster; }
    protected boolean isTargetFound() { return targetCluster != null; }
}