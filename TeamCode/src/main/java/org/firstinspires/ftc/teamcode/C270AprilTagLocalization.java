package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * LinearOpMode for testing AprilTag localization using a Logitech C270 USB Webcam.
 * Detects tags (including Hive / Game AprilTags) and calculates estimated Robot Field Pose.
 */
@TeleOp(name = "C270 AprilTag Localization", group = "Vision")
public class C270AprilTagLocalization extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // CAMERA OFFSET CONFIGURATION
    // Adjust these values to match where your Logitech C270 is mounted on the robot chassis.
    // ----------------------------------------------------------------------------------
    // Position: (X = Left/Right inches, Y = Forward/Back inches, Z = Up/Down inches from robot center)

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    private Position cameraPosition = new Position(
            DistanceUnit.INCH, 0.0, 0.0, 6.0, 0
    );

    // Orientation: Yaw (0 = facing forward, +90 = facing left), Pitch (-90 = horizontal camera), Roll
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES, 0.0, -70.0, 0.0, 0
    );

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize AprilTag Processor and VisionPortal for Logitech C270
        initAprilTag();

        telemetry.addData("Status", "Initialized. Point camera at AprilTags.");
        telemetry.addData("Webcam", "Logitech C270 ('Webcam 1')");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Display AprilTag detections and estimated robot pose
            telemetryAprilTag();
            telemetry.update();

            // Save CPU resources
            sleep(20);
        }

        // Close VisionPortal when OpMode finishes
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize AprilTag Processor and VisionPortal with C270 camera parameters.
     */
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // Calibration Intrinsics for Logitech C270 at 640x480 resolution (fx, fy, cx, cy)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Hardware map name for C270 webcam configured on the Robot Controller
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Enable AprilTag processor
        builder.addProcessor(aprilTag);

        // Build VisionPortal instance
        visionPortal = builder.build();
    }

    /**
     * Parse detected AprilTags and display tag positions & estimated Robot Pose on Driver Station.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n=== Tag ID %d: %s ===", singleDet.id, singleDet.metadata.name));

                    // Calculated Robot Field Pose (if tag is in tag library)
                    if (detection.robotPose != null) {
                        telemetry.addLine(String.format("Robot Field Pose (X, Y, Z): %6.1f, %6.1f, %6.1f in",
                                detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getPosition().z));

                        telemetry.addLine(String.format("Robot Heading (Yaw): %6.1f deg",
                                detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    }

                    // Relative Tag Position from Camera
                    if (detection.ftcPose != null) {
                        telemetry.addLine(String.format("Tag Relative (Range, Bearing): %6.1f in, %6.1f deg",
                                detection.ftcPose.range,
                                detection.ftcPose.bearing));
                    }
                } else {
                    telemetry.addLine(String.format("\n=== Tag ID %d: Unknown Tag ===", singleDet.id));
                    if (singleDet.center != null) {
                        telemetry.addLine(String.format("Screen Center: (%6.0f, %6.0f) px",
                                singleDet.center.x, singleDet.center.y));
                    }
                }
            }
        }

        if (currentDetections.isEmpty()) {
            telemetry.addLine("\nNo AprilTags visible.");
        }
    }
}
