package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

// import org.firstinspires.ftc.robotcore.external.Telemetry;

// Linear Opmode is for sequential code like in Autonomous operation
// (doing things in a linear fashion, one after the other.

// OpMode is for looping behavior during TeleOp

// @TeleOp puts it in the TeloOp menu on the driver station
// group can be used to further subdivide the OpMode in the driver station.
@TeleOp(name="TeleopBallLaunch", group="TeleOp Basic")
// Code taken from 2024 robot and cleaned up as starting point for 2025.
public class TeleopBallLaunch extends OpMode {
    private Robot robot;
    public final double VERSION = 1;
    final ElapsedTime runtime = new ElapsedTime();
    // Declare OpMode members for each of the 4 motors.
    private double timeOfLastFlywheelUpdate = runtime.time();
    final double minTimeBetweenFlywheelUpdates = 0.2;

    final double maxLaunchSpeed = 1;
    final double minLaunchSpeed = 0.4;

    double launchPower = minLaunchSpeed;
    final double getLaunchSpeedIncrement = 0.01;

    final double servoFeedSpeed = 0.5;
    double driveSpeed = 0.7;
    double turnSpeed = 0.6;
    private final Object timer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

        runtime.reset();
        telemetry.addData("Status", "Initialized for TeleOp");
        telemetry.addData("Status (Version: " + VERSION + ") Initialized", "Run Time: " + runtime);

        telemetry.update();
    }

    @Override
    public void init_loop() {

        telemetry.addData("Status (Version: " + VERSION + ")", "Run Time: " + runtime);
        telemetry.addData("Status", "Initialized for TeleOp (INIT LOOP)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.flywheel.setPower(launchPower);

        telemetry.addData("Status (Version: " + VERSION + ")", "Run Time: " + runtime);
        telemetry.addData("Status", "Started for TeleOp (START)");
        telemetry.update();
    }

    private void launchBall() {
        robot.leftFeed.setPower(-servoFeedSpeed);
        robot.rightFeed.setPower(servoFeedSpeed);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
       // robot.leftFeed.setPower(0);
        // robot.rightFeed.setPower(0);
       // telemetry.addData()
    }

    private void adjustFlyheelSpeed(boolean increaseSpeed) {
        double currentTime = runtime.time();

        if (currentTime - timeOfLastFlywheelUpdate >= minTimeBetweenFlywheelUpdates) {
            timeOfLastFlywheelUpdate = currentTime;
            if (increaseSpeed) {
                launchPower += getLaunchSpeedIncrement;
            } else {
                launchPower -= getLaunchSpeedIncrement;
            }
            if (launchPower < minLaunchSpeed) {
                launchPower = minLaunchSpeed;
            }
            if (launchPower > maxLaunchSpeed) {
                launchPower = maxLaunchSpeed;
            }
            telemetry.addData("Launch speed changed at time" + currentTime, "to %4.2f", launchPower);
            robot.flywheel.setPower(launchPower);
        }
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            adjustFlyheelSpeed(true);
        } else if (gamepad1.dpad_down) {
            adjustFlyheelSpeed(false);
        }
        if (gamepad1.a) {
            robot.flywheel.setPower(0);
        }
        if (gamepad1.y) {
            robot.flywheel.setPower(minLaunchSpeed);
        }

        if (gamepad1.b) {
            launchBall();
        }

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = driveSpeed * (axial + lateral) + turnSpeed * yaw;
        double rightFrontPower = driveSpeed * (axial - lateral) - turnSpeed * yaw;
        double leftBackPower = driveSpeed * (axial - lateral) + turnSpeed * yaw;
        double rightBackPower = driveSpeed * (axial + lateral) - turnSpeed * yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.
        leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

        // Send calculated power to wheels

        /* Prevent wheels from moving
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightBackDrive.setPower(rightBackPower);
        */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status (Version: " + VERSION + ")", "Run Time: " + runtime);
        telemetry.addData("Launch speed:", "%4.2f", launchPower);

        telemetry.update();
    }
}




