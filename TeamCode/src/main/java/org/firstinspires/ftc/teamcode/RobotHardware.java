package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    HardwareMap hardwareMap;

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    DcMotor armLift;

    Servo armRotate;
    Servo clawGrab;

    RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");

        armLift = hardwareMap.get(DcMotor.class, "al");

        armRotate = hardwareMap.get(Servo.class, "ar");
        clawGrab = hardwareMap.get(Servo.class, "cg");

        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        armLift.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();
    }

    public void resetDriveEncoders() {
        brake();

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drivePower(double blPower, double brPower, double flPower, double frPower) {
        backLeftDrive.setPower(Range.clip(blPower, -1, 1));
        backRightDrive.setPower(Range.clip(brPower, -1, 1));
        frontLeftDrive.setPower(Range.clip(flPower, -1, 1));
        frontRightDrive.setPower(Range.clip(frPower, -1, 1));
    }

    public void drive(double drive, double strafe, double rotate, double speed) {
        final double blPower = speed * (drive - rotate + strafe);
        final double brPower = speed * (drive + rotate - strafe);
        final double flPower = speed * (drive - rotate - strafe);
        final double frPower = speed * (drive + rotate + strafe);
        drivePower(blPower, brPower, flPower, frPower);
    }

    public void brake() {
        drivePower(0, 0, 0, 0);
    }

    public void armLiftPower(double power) {
        armLift.setPower(power);
    }

    public void rotateArm(double to) {
        armRotate.setPosition(to);
    }

    public void grabClaw(double to) {
        clawGrab.setPosition(to);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
