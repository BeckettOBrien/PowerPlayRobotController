package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.misc.Encoder;

@Config
public class RobotHardware {

    public static double ARM_FORWARD = 0.115;
    public static double ARM_BACKWARD = 0.84;

    public static double CLAW_OPEN = 0.125;
    public static double CLAW_CLOSED = 0.24;

    public static double ARM_SPEED_MULTIPLIER = 0.85;
    public static int MAX_LIFT_DEADZONE = 25;
    public static int MIN_LIFT_DEADZONE = 15;
    public static double MAX_TOP_LIFT_POWER = 0.025;
    public static double MAX_BOT_LIFT_POWER = -0.075;
    public static double MIN_ARM_ROTATE_HEIGHT = 600;
    public static double HIGHEST_STABLE_LIFT_POS = 700;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder backEncoder;

    public DcMotorEx armLift;
    public DigitalChannel armLimit;

    public Servo armRotate;
    public Servo clawGrab;

    static int lowestArmPosition = 0;
    static int highestArmPosition = 3100;
    static int currentArmPosition = 0;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        telemetry = FtcDashboard.getInstance().getTelemetry();

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bld");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frd");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));

        armLift = hardwareMap.get(DcMotorEx.class, "al");
        armLimit = hardwareMap.get(DigitalChannel.class, "armLimit");

        armRotate = hardwareMap.get(Servo.class, "ar");
        clawGrab = hardwareMap.get(Servo.class, "cg");

        backLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        backEncoder.setDirection(Encoder.Direction.FORWARD);

        armLift.setDirection(DcMotorEx.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lowestArmPosition = 0;
        currentArmPosition = armLift.getCurrentPosition();

//        resetEncoders();
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

    public void setLift(double percent) {
        if ((Math.abs(percent) < 0.1) && armLimit.getState()) {
            updateLift();
        } else {
            liftPower(percent);
        }
    }

    public void liftPower(double percent) {
        double power = percent * ARM_SPEED_MULTIPLIER;
        if (!armLimit.getState() || (armLift.getCurrentPosition() >= (highestArmPosition - MAX_LIFT_DEADZONE))) {
            power = Range.clip(power, -1, MAX_TOP_LIFT_POWER);
        }
        if (armLift.getCurrentPosition() <= (lowestArmPosition + MIN_LIFT_DEADZONE)) {
            power = Range.clip(power, MAX_BOT_LIFT_POWER, 1);
        }
        FtcDashboard.getInstance().getTelemetry().addData("liftPower", power);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift.setPower(power);
        currentArmPosition = armLift.getCurrentPosition();
    }

    public void liftTo(int target) {
        armLift.setTargetPosition(target);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(0.5);
        while (armLift.isBusy() || (Math.abs(armLift.getCurrentPosition() - armLift.getTargetPositionTolerance()) < 20)) sleep(1);
        currentArmPosition = target;
        armLift.setPower(0.15);
    }

    public void calibrateLift() {
        while (armLimit.getState()) {
            armLift.setPower(0.5);
        }
        highestArmPosition = armLift.getCurrentPosition();
        armLift.setPower(0);
    }

    public void updateLift() {
//        armLift.setTargetPosition();
        telemetry.addData("Target arm position", currentArmPosition);
        if ((currentArmPosition > HIGHEST_STABLE_LIFT_POS) && (currentArmPosition < (highestArmPosition - MAX_LIFT_DEADZONE))) {
            armLift.setTargetPosition(currentArmPosition);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(0.15);
        } else {
            armLift.setPower(0);
        }
    }

    public void rotateArm(double to) {
        if (armLift.getCurrentPosition() < MIN_ARM_ROTATE_HEIGHT) return;
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
