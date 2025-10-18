/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// Directory of package
package org.firstinspires.ftc.teamcode;

// Declare imports
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// import org.firstinspires.ftc.robotcore.external.Telemetry;

// Linear Opmode is for sequential code like in Autonomous operation
// (doing things in a linear fashion, one after the other.

// OpMode is for looping behavior during TeleOp

// @TeleOp puts it in the TeloOp menu on the driver station
// group can be used to further subdivide the OpMode in the driver station.
@TeleOp(name="TeleopWithoutAutoFrom2024", group="TeleOp Basic")
// Code taken from 2024 robot and cleaned up as starting point for 2025.
public class TeleopWithoutAutoFrom2024 extends OpMode {
    public final double VERSION = 1;
    final ElapsedTime runtime = new ElapsedTime();
    // Declare OpMode members for each of the 4 motors.
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    double driveSpeed = 0.7;
    double turnSpeed = 0.6;

    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (t     he wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status (Version: " + VERSION + ") Initialized", "Run Time: " + runtime);

        telemetry.update();
    }

    @Override
    public void loop() {

        // Set mode for motors

        // winch.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)


        double max;

        // Speed control with dpad. Like a knob where the top is highest.
        if (gamepad1.dpad_up) {
            driveSpeed = 1;
            turnSpeed = 1;
        } else if (gamepad1.dpad_right) {
            driveSpeed = 0.1;
            turnSpeed = 0.1;
        } else if (gamepad1.dpad_down) {
            driveSpeed = 0.4;
            turnSpeed = 0.3;
        } else if (gamepad1.dpad_left) {
            driveSpeed = 0.7;
            turnSpeed = 0.6;
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
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
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

        /*
        leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
        */

        // Send calculated power to wheels

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status (Version: " + VERSION + ")", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
}




