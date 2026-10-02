package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class FIRST_Teleop extends LinearOpMode {
    private Follower follower;
    private DcMotor intake;
    private boolean intakeOn = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower (handles drive motors + odometry tracking)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized with Odometry");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Optional: Add a deadzone so the robot doesn't creep
            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(strafe) < 0.05) strafe = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            // Drive field-centric (false = field-centric, true = robot-centric)
            follower.setTeleOpDrive(forward, strafe, turn, false);

            // Update Pedro Pathing odometry localization and drive motor outputs
            follower.update();

            // Press Options or B to reset odometry position and heading back to (0, 0, 0)
            if (gamepad1.options || gamepad1.b) {
                follower.setPose(new Pose(0, 0, 0));
            }

            // Intake toggle: react on the moment A is first pressed
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastA) {
                intakeOn = !intakeOn;
            }
            lastA = aPressed;

            intake.setPower(intakeOn ? 1.0 : 0.0);

            // Fetch active odometry coordinates
            Pose currentPose = follower.getPose();

            // Target Goal & Shooter Physics Constants
            double goalX = 72.0;            // Target goal X position (inches)
            double goalY = 144.0;           // Target goal Y position (inches)
            double goalHeightIn = 40.0;     // Target goal height (inches)
            double launchHeightIn = 15.0;   // Launcher height on robot (inches)

            double m = 0.05;                // Projectile mass (kg)
            double s = 0.15;                // Contact barrel length (m)
            double I = 0.0005;              // Flywheel moment of inertia (kg*m^2)
            double alpha = 500.0;           // Angular acceleration (rad/s^2)
            double r = 0.05;                // Flywheel radius (m)

            // Dynamic distance calculation from active odometry pose to target goal
            double dx = goalX - currentPose.getX();
            double dy = goalY - currentPose.getY();
            double distanceInches = Math.hypot(dx, dy);

            // Unit conversions to meters & radians
            double d = distanceInches * 0.0254;                   // convert inches to meters
            double h = (goalHeightIn - launchHeightIn) * 0.0254;  // height difference in meters
            double theta = Math.toRadians(45);                    // 45 degree launch angle

            // Calculate required launch velocity and total shooter force using ShooterPhysics
            double reqVelocity = ShooterPhysics.calculateLaunchVelocity(d, h, theta);
            double F = ShooterPhysics.totalForce(m, s, d, h, theta, I, alpha, r);

            // Display Odometry coordinates, status, and shooter physics on Driver Station screen
            telemetry.addData("Status", "Running");
            telemetry.addData("X (Inches)", "%.2f", currentPose.getX());
            telemetry.addData("Y (Inches)", "%.2f", currentPose.getY());
            telemetry.addData("Heading (Deg)", "%.2f", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Goal Dist (in)", "%.2f", distanceInches);
            telemetry.addData("Req Launch Vel (m/s)", "%.2f", reqVelocity);
            telemetry.addData("Shooter Force (N)", "%.2f", F);
            telemetry.update();

        }
    }
}


