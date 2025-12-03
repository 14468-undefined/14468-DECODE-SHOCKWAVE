package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Comparator;
import java.util.List;


@Config
public class WebcamVisionSubsystem extends UndefinedSubsystemBase {

    public final VisionPortal webcam;
    private final AprilTagProcessor aprilTagProcessor;

    private AprilTagDetection desiredTag;

    private static final double DESIRED_DISTANCE = 12.0;

    public static double P_DIST = 0;//..08
    public static double I_DIST = 0;//.0008
    public static double D_DIST = 0;//.025

    public static double P_HEADING = 0.04;//.02
    public static double I_HEADING = 0.0;//.0005
    public static double D_HEADING = 0.008;//.005

    public static double P_STRAFE = 0;//.48
    public static double I_STRAFE = 0.0;//0
    public static double D_STRAFE = 0;//.12

    public static double MAX_SPEED = 0.8;//.5
    public static double MAX_TURN = 0.5;//.3
    public static double MAX_STRAFE = 0.8;//.5

    private double distIntegral = 0, lastDistError = 0;
    private double headingIntegral = 0, lastHeadingError = 0;
    private double strafeIntegral = 0, lastStrafeError = 0;

    public WebcamVisionSubsystem(HardwareMap hwMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        webcam = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)

                .build();

    }

    public boolean hasTarget() {
        return desiredTag != null;
    }

    public boolean isAtTarget() {
        if (!hasTarget()) return false;
        return Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE) < 0.5;
    }

    @Override
    public void periodic() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        desiredTag = detections.stream()
                .min(Comparator.comparingDouble(d -> d.ftcPose.range))
                .orElse(null);
    }

    public CommandBase getDriveToTagCommand(DriveSubsystem drive) {
        return this.runEnd(() -> {

            if (!hasTarget()) {
                drive.rest();
                resetPID();
                return;
            }

            double distError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double strafeError = desiredTag.ftcPose.x;

            double forward = computeDistPID(distError);
            double turn = computeHeadingPID(headingError);
            double strafe = computeStrafePID(strafeError);

            forward = clip(forward, -MAX_SPEED, MAX_SPEED);
            turn = clip(turn, -MAX_TURN, MAX_TURN);
            strafe = clip(strafe, -MAX_STRAFE, MAX_STRAFE);

            drive.drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(forward, strafe), turn)
            );

        }, drive::rest);
    }

    private double computeDistPID(double error) {
        distIntegral += error;
        double derivative = error - lastDistError;
        lastDistError = error;
        return P_DIST * error + I_DIST * distIntegral + D_DIST * derivative;
    }

    private double computeHeadingPID(double error) {
        headingIntegral += error;
        double derivative = error - lastHeadingError;
        lastHeadingError = error;
        return P_HEADING * error + I_HEADING * headingIntegral + D_HEADING * derivative;
    }

    private double computeStrafePID(double error) {
        strafeIntegral += error;
        double derivative = error - lastStrafeError;
        lastStrafeError = error;
        return P_STRAFE * error + I_STRAFE * strafeIntegral + D_STRAFE * derivative;
    }

    private void resetPID() {
        distIntegral = headingIntegral = strafeIntegral = 0;
        lastDistError = lastHeadingError = lastStrafeError = 0;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine("Tag Detected: " + hasTarget());
        if (hasTarget()) {
            t.addLine("Tag ID: " + desiredTag.id);
            t.addLine(String.format(
                    "X: %.2f  Y: %.2f  Range: %.2f",
                    desiredTag.ftcPose.x,
                    desiredTag.ftcPose.y,
                    desiredTag.ftcPose.range
            ));
        }
    }
}
