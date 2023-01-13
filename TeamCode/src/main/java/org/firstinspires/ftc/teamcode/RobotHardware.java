package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class RobotHardware {

    public static double ARM_FORWARD = 0.05;
    public static double ARM_BACKWARD = 0.74;

    public static double CLAW_OPEN = 0.2;
    public static double CLAW_CLOSED = 0.46;

    public static double ARM_SPEED_MULTIPLIER = 0.85;
    public static int MAX_LIFT_HEIGHT = 2100;

    HardwareMap hardwareMap;

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    public DcMotor armLift;

    public Servo armRotate;
    public Servo clawGrab;

    public RobotHardware(HardwareMap hardwareMap) {
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

        armLift.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void resetEncoders() {
        brake();

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
//            armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armLift.setPower(power);
//        } else if (power < 0) {
//            armLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armLower.setPower(power);
//            armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armLift.setPower(-power/8);
//        } else if (armLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            armLift.setTargetPosition(armLift.getCurrentPosition());
//            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armLift.setPower(0.2);
//        } else {
//            armLower.setPower(0);
//        }
//    }

//    public void armLowerPower(double power) {
//        armLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armLower.setPower(power);
//        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
