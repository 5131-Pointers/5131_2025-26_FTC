package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "Comp1 Auto", group = "Autonomous")
public class Comp1Auto extends LinearOpMode {
    public class Outtake {
        private CRServo outtakeRotate;
        private DcMotorEx SpinMotor;
        private DcMotorEx MidIntake;
        private DcMotorEx FrontIntake;
        public double SpinSpeed = 0;
        private static final double COUNTS_PER_REV = 27;
        private static final double MAX_RPM = 6000.0;
        private static final double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * COUNTS_PER_REV; // 11200 tps

        public Outtake(HardwareMap hardwareMap) {
            outtakeRotate = hardwareMap.get(CRServo.class, "outtakeRotate");
            SpinMotor = hardwareMap.get(DcMotorEx.class, "SpinMotor");
            MidIntake = hardwareMap.get(DcMotorEx.class, "MidIntake");
            FrontIntake = hardwareMap.get(DcMotorEx.class, "FrontIntake");
            SpinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SpinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class outtakeSpin implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpinSpeed = 0.8;
                double targetTicksPerSec = SpinSpeed * MAX_TICKS_PER_SEC;
                SpinMotor.setVelocity(-targetTicksPerSec);


                return false;
            }
        }
        public Action outtakeSpin() {

            return new outtakeSpin();
        }

        public class intakeBalls implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeRotate.setPower(1);
                MidIntake.setPower(-1);
                FrontIntake.setPower(1);
                return false;
            }
        }
        public Action intakeBalls() {
            return new intakeBalls();
        }
        public class stopIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeRotate.setPower(0);
                MidIntake.setPower(0);
                FrontIntake.setPower(0);
                return false;
            }
        }
        public Action stopIntake() {
            return new stopIntake();
        }
        public class stopSpin implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpinSpeed = 0;
                SpinMotor.setVelocity(0);
                return false;
            }
        }
        public Action stopSpin() {
            return new stopSpin();
        }



    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Rotate rotate = new Rotate(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);


        // vision here that outputs position
        //int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(2);
                //.strafeToLinearHeading(new Vector2d(15.5, 12), Math.toRadians(135));
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .waitSeconds(4);
                //.strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(135));


        Action trajectoryActionCloseOut = tab2.endTrajectory().fresh()
                .waitSeconds(2)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(outtake.outtakeSpin());
        //Actions.runBlocking(claw.wristStartPos());








        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        //Actions.runBlocking(rotate.rotateGrabPos());
        //while (!isStopRequested() && !opModeIsActive()) {
        //    int position = visionOutputPosition;
        //    telemetry.addData("Position during Init", position);
        //    telemetry.update();
        //}

        //int startPosition = visionOutputPosition;
        //telemetry.addData("Starting Position", startPosition);
        //telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        //Action trajectoryActionChosen;
        //trajectoryActionChosen = tab1.build();


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        outtake.intakeBalls(),
                        tab2.build(),
                        outtake.stopIntake(),
                        outtake.stopSpin(),
                        trajectoryActionCloseOut

                )
        );
    }
}