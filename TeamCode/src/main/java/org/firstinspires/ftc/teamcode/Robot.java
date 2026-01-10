package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

enum Direction {
    LEFT, RIGHT
}

public class Robot {
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotorEx flywheel = null;
    public DcMotor lift = null;

    public CRServo leftFeed = null;
    public CRServo rightFeed = null;

    public final int DEFAULT_FEED_DURATION = 350;
    //The ball delays in the beginning so this is so it could run as similar to the rest
    public final int FIRST_LAUNCH_DURATION = DEFAULT_FEED_DURATION + 200;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    final double DRIVE_WHEEL_DIAMETER = 104/25.4;
    final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;
    //TPR is the rev. of the wheel motors
    final double TICKS_PER_REVOLUTION = 537.7;

    final double servoFeedSpeed = 1.0;

    double DEFAULT_LIFT_POWER = 0.5;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lift = hardwareMap.get(DcMotor.class, "lift0");
        lift.setDirection(DcMotor.Direction.FORWARD);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFeed = hardwareMap.get(CRServo.class, "left_feed");
        rightFeed = hardwareMap.get(CRServo.class, "right_feed");


        double F = 13.2; // Feedforward gain to counteract constant forces like friction.
        double P = 265;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        // Apply the new coefficients to the motor in every loop iteration.
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    public void stopAllDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void driveUsingEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveWithoutEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setFlywheelPower(double launchPower){
        telemetry.addData("Launch speed changed", "to %4.2f", launchPower);
        flywheel.setPower(launchPower);
    }
    public void autoPosition(double desiredDistance){
        double TargetTicks;
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TargetTicks = (desiredDistance / DRIVE_WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

        //leftFrontDrive.setTargetPosition((int) (leftFrontDrive.getCurrentPosition() ))
    }
    public void launchBall(int duration) {
        leftFeed.setPower(-servoFeedSpeed);
        rightFeed.setPower(-servoFeedSpeed);
        try {
            sleep(duration);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        leftFeed.setPower(0);
        rightFeed.setPower(0);
       /* try {
            //sleep(500);
       // } catch (InterruptedException e) {
          //  throw new RuntimeException(e);
        } */
        //telemetry.addData();
    }
    public void liftOn(boolean Up){
        double liftPower = DEFAULT_LIFT_POWER;
        if (Up) {
            liftPower = -liftPower;
        }
        telemetry.addData("Lift power", "to %4.2f", liftPower);
        lift.setPower(liftPower);
    }

    public void liftOff(){
        double liftPower = 0;
        telemetry.addData("Lift power", "to %4.2f", liftPower);
        lift.setPower(liftPower);
    }
    private double calculateRPM(DcMotorEx motor, double TICKS_PER_REV) {
        double ticksPerSecond = motor.getVelocity(); // Get velocity from the motor controller
        double rotationsPerSecond = ticksPerSecond / TICKS_PER_REV;
        double rpm = rotationsPerSecond * 60.0; // Convert to rotations per minute
        return rpm;
    }
    public void calculateFlywheelSpeed() {
        double currentRPM = calculateRPM(flywheel, 28);
        telemetry.addData("Motor RPM", "%.2f", currentRPM);
        //telemetry.update();
    }
    public void setTargetVelocity(double targetRPM) {
        double targetVelocity = targetRPM * 28/60;
        flywheel.setVelocity(targetVelocity);
        telemetry.addData("Target Velocity ", "%.2f", targetVelocity);
        telemetry.addData("Target RPM ", "%.2f", targetRPM);

    }

}
