package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.*;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


@TeleOp(name = "Comp1 Mecanum Drive", group = "Linear Opmode")
public class Comp1MecanumDrive extends LinearOpMode {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;
    public DcMotor FrontIntake;
    public DcMotor MidIntake;
    public DcMotorEx SpinMotor;

    public CRServo outtakeRotate;

    public double SpinSpeed = 0;
    MecanumDrive drive;
    private static final double COUNTS_PER_REV = 27;
    private static final double MAX_RPM = 6000.0;
    private static final double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * COUNTS_PER_REV; // 11200 tps


    @Override
    public void runOpMode() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        FrontIntake = hardwareMap.dcMotor.get("FrontIntake");
        MidIntake = hardwareMap.dcMotor.get("MidIntake");
        SpinMotor = hardwareMap.get(DcMotorEx.class, "SpinMotor");
        outtakeRotate = hardwareMap.crservo.get("outtakeRotate");

        SpinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));




        ExecutorService executorService = Executors.newFixedThreadPool(2);
        executorService.submit(this::mecanumDrive);
        executorService.submit(this::otherMovement);


        waitForStart();

        while (opModeIsActive()) {
            //Drive();
            //intakeShoot();

            double currentVelocity = SpinMotor.getVelocity(); // ticks per second

            telemetry.addData("SpinSpeed (0-1)", SpinSpeed);
            telemetry.addData("Encoder Count: ", SpinMotor.getCurrentPosition());

            telemetry.addData("Spin Target tps", SpinSpeed * MAX_TICKS_PER_SEC);
            telemetry.addData("Spin Actual tps", currentVelocity);
            telemetry.update();
        }
        executorService.shutdownNow(); // Shut down the ExecutorService

    }
    private void mecanumDrive() {
        while (!Thread.interrupted()) {
            Drive();

        }
    }
    private void otherMovement() {
        while (!Thread.interrupted()) {
            intakeShoot();

        }
    }
    public void Drive() {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();
    }

    public void intakeShoot() {
        if (gamepad1.a) {
            SpinSpeed += 0.005;
            sleep(15);
        }
        if (gamepad1.y) {
            SpinSpeed -= 0.005;
            sleep(15);
        }

        if (gamepad1.right_bumper) {
            FrontIntake.setPower(1);
        }
        else {
            FrontIntake.setPower(0);
        }
        if (gamepad1.left_bumper) {
            MidIntake.setPower(-1);
        }
        else {
            MidIntake.setPower(0);
        }

        if (gamepad1.x) {
            outtakeRotate.setPower(1);
        } else if (gamepad1.b) {
            outtakeRotate.setPower(-1);
        } else {
            outtakeRotate.setPower(0);
        }

        if (gamepad2.x) {
            MidIntake.setPower(1);
        }
        if (gamepad2.y) {
            FrontIntake.setPower(-1);
        }
        if (gamepad2.right_bumper) {
            SpinSpeed = 0.9;
            telemetry.addData("Shoot: ", "FAR");
            telemetry.update();
            //far shoot
        }

        if (gamepad2.left_bumper) {
            SpinSpeed = 0.6;
            telemetry.addData("Shoot: ", "CLOSE");
            telemetry.update();
            //close shoot shoot
        }

        SpinSpeed = Range.clip(SpinSpeed, 0.0, 1.0);
        double targetTicksPerSec = SpinSpeed * MAX_TICKS_PER_SEC;
        SpinMotor.setVelocity(-targetTicksPerSec);
    }
    }


