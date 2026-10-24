package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * LinearOpMode Test File for Buzzy Launcher Tuning with FTC Dashboard support.
 * - View live plots at http://192.168.43.1:8080/dash
 * - Adjust TARGET_VELOCITY on the fly via FTC Dashboard or Gamepad Dpad Up/Down.
 */
@Config
@TeleOp(name = "Buzzy_Launcher_Test", group = "Test")
public class BuzzyTest extends LinearOpMode {

    // Hardware members
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private VoltageSensor batteryVoltageSensor = null;
    private IMU imu = null;

    // Launcher Tuning & Controls (Editable live in FTC Dashboard!)
    public static int TARGET_VELOCITY = 1250; // Initial target velocity (ticks/sec)
    public static int VELOCITY_STEP = 25;      // Ticks/sec adjustment step
    public static double VELOCITY_TOLERANCE_PCT = 0.03;

    // Edge detection variables for buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;

    // Drive & Intake power variables
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double intakePower;

    @Override
    public void runOpMode() {
        // Initialize FTC Dashboard Telemetry (sends telemetry to both Driver Station and Web Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ----------------------------------------------------------------------------------
        // HARDWARE INITIALIZATION
        // ----------------------------------------------------------------------------------
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        try {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            batteryVoltageSensor = null;
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized. View FTC Dashboard at 192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        // ----------------------------------------------------------------------------------
        // MAIN OPMODE LOOP
        // ----------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // Reset IMU heading if Options is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Field-Oriented Mecanum Drive
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Intake Power from triggers
            intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

            // Target Velocity Adjustment (Dpad Up / Dpad Down)
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                TARGET_VELOCITY += VELOCITY_STEP;
            }
            if (currentDpadDown && !lastDpadDown) {
                TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - VELOCITY_STEP);
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // Handle Launching & Voltage Compensation
            launch();

            // Set intake motor & servo powers
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            // Live FTC Dashboard Plotting & Telemetry
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Actual Velocity", launcher.getVelocity());
            telemetry.addData("Velocity Error", TARGET_VELOCITY - launcher.getVelocity());
            telemetry.addData("Ready to Launch", isReadyToLaunch() ? 1 : 0);
            if (batteryVoltageSensor != null) {
                telemetry.addData("Battery Voltage", batteryVoltageSensor.getVoltage());
            }
            telemetry.addData("Heading Deg", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            telemetry.update();
        }
    }

    private boolean isReadyToLaunch() {
        double currentVel = launcher.getVelocity();
        double error = Math.abs(TARGET_VELOCITY - currentVel);
        return error <= (TARGET_VELOCITY * VELOCITY_TOLERANCE_PCT);
    }

    private void launch() {
        boolean currentRightBumper = gamepad1.right_bumper;

        // Sample resting voltage & update PIDF ONCE when right bumper is first pressed
        if (currentRightBumper && !lastRightBumper && batteryVoltageSensor != null) {
            double restVoltage = batteryVoltageSensor.getVoltage();
            if (restVoltage > 0) {
                double nominalVoltage = 12.0;
                double baseF = 12.5;
                double compensatedF = baseF * (nominalVoltage / restVoltage);

                launcher.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, compensatedF)
                );
            }
        }
        lastRightBumper = currentRightBumper;

        if (currentRightBumper) {
            launcher.setVelocity(TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (currentRightBumper && isReadyToLaunch()) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        leftFrontPower  = rotY + rotX + rotate;
        rightFrontPower = rotY - rotX - rotate;
        leftBackPower   = rotY - rotX + rotate;
        rightBackPower  = rotY + rotX - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
