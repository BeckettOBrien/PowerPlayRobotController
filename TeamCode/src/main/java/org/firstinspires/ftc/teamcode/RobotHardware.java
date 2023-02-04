package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

@Config
public class RobotHardware {

    public static double ARM_FORWARD = 0.22;
    public static double ARM_BACKWARD = 0.95;

    public static double CLAW_OPEN = 0.1;
    public static double CLAW_CLOSED = 0.25;

    public static double ARM_SPEED_MULTIPLIER = 0.85;
    public static int MAX_LIFT_HEIGHT = 2100;

    HardwareMap hardwareMap;

    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public DcMotorEx armLift;

    public Servo armRotate;
    public Servo clawGrab;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bld");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frd");

        armLift = hardwareMap.get(DcMotorEx.class, "al");

        armRotate = hardwareMap.get(Servo.class, "ar");
        clawGrab = hardwareMap.get(Servo.class, "cg");

        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        armLift.setDirection(DcMotorEx.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void resetEncoders() {
        brake();

        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void drivePower(double blPower, double brPower, double flPower, double frPower) {
        backLeftDrive.setPower(Range.clip(blPower, -1, 1));
        backRightDrive.setPower(Range.clip(brPower, -1, 1));
        frontLeftDrive.setPower(Range.clip(flPower, -1, 1));
        frontRightDrive.setPower(Range.clip(frPower, -1, 1));
    }

    public void drive(double drive, double strafe, double rotate, double speed) {
        final double blPower = speed * (drive - rotate - strafe);
        final double brPower = speed * (drive + rotate + strafe);
        final double flPower = speed * (drive - rotate + strafe);
        final double frPower = speed * (drive + rotate - strafe);
        drivePower(blPower, brPower, flPower, frPower);
    }

    public void brake() {
        drivePower(0, 0, 0, 0);
    }

//    public void armLiftPower(double power) {
//        if (power > 0) {
//            armLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            armLift.setPower(power);
//        } else if (power < 0) {
//            armLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            armLower.setPower(power);
//            armLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            armLift.setPower(-power/8);
//        } else if (armLift.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
//            armLift.setTargetPosition(armLift.getCurrentPosition());
//            armLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            armLift.setPower(0.2);
//        } else {
//            armLower.setPower(0);
//        }
//    }

//    public void armLowerPower(double power) {
//        armLower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        armLower.setPower(power);
//        armLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        armLift.setPower(power/2);
//    }

//    public void setArmHeight(double percent) {
//        int target = (int)Range.clip(percent * MAX_LIFT_HEIGHT, 0, MAX_LIFT_HEIGHT);
//        armLift.setTargetPosition(target);
//        if (target > armLift.getCurrentPosition()) {
//            armLower.setPower(0);
//            armLift.setPower(0.5);
//        } else {
//            armLift.setPower(0.3);
//            armLower.setPower(0.25);
//        }
//    }

    public void armPower(double percent) {
        armLift.setPower(percent * ARM_SPEED_MULTIPLIER);
    }

    public void rotateArm(double to) {
        armRotate.setPosition(to);
    }

    public void grabClaw() {
        clawGrab.setPosition(CLAW_CLOSED);
    }

    public void releaseClaw() {
        clawGrab.setPosition(CLAW_OPEN);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
