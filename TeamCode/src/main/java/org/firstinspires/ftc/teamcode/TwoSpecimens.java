package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Two Specimens")
public class TwoSpecimens extends LinearOpMode {

    public class ScoringClaw{
        private Servo claw;
        public ScoringClaw(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "scoring_claw");
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Constants.SCORING_OPEN);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Constants.SCORING_CLOSE);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }

    public class ScoringArm{
        private Servo arm_right;
        private Servo arm_left;

        public ScoringArm(HardwareMap hardwareMap){
            arm_right = hardwareMap.get(Servo.class, "scoring_arm_right");
            arm_left = hardwareMap.get(Servo.class, "scoring_arm_left");
        }

        public class Home implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm_left.setPosition(Constants.SCORING_ARM_HOME);
                arm_right.setPosition(1 - Constants.SCORING_ARM_HOME);
                return false;
            }
        }
        public Action home() {
            return new Home();
        }

        public class Bar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm_left.setPosition(Constants.SCORING_ARM_BASKET);
                arm_right.setPosition(1 - Constants.SCORING_ARM_BASKET);
                return false;
            }
        }
        public Action bar() {
            return new Bar();
        }
    }

    public class ScoringSlides{
        private DcMotorEx rightSlide;
        private DcMotorEx leftSlide;

        private PController controller;

        public ScoringSlides(HardwareMap hardwareMap){
            rightSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "R_slide");
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "L_slide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            controller=new PController(.003);
            controller.setTolerance(35);

        }

        public class Home implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                controller.setSetPoint(Constants.SLIDE_HOME);
                double power = controller.calculate(leftSlide.getCurrentPosition());
                leftSlide.setPower(power);
                rightSlide.setPower(power);

                return !controller.atSetPoint();
            }
        }
        public Action home() {
            return new Home();
        }

        public class Bar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                controller.setSetPoint(Constants.SLIDE_BAR);
                double power = controller.calculate(leftSlide.getCurrentPosition());
                leftSlide.setPower(power);
                rightSlide.setPower(power);

                return !controller.atSetPoint();
            }
        }
        public Action bar() {
            return new Bar();
        }
    }


    @Override
public void runOpMode(){
        /* X+ is to the right
        *  Y+ is away from you
        *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(0, -63.5, Math.toRadians(270)); // START WITH SCOREING CLAW FACEING SUB
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);
        ScoringClaw scoringClaw = new ScoringClaw(hardwareMap);
        ScoringArm scoringArm = new ScoringArm(hardwareMap);
        ScoringSlides scoringslides = new ScoringSlides(hardwareMap);

TrajectoryActionBuilder SCORE = drive.actionBuilder(startpose)
        .lineToY(-38);
        scoringArm.bar(),
                scoringslides.bar(),

        TrajectoryActionBuilder safe = park.endTrajectory().fresh()
            .lineToY(-50);

        Actions.runBlocking(new ParallelAction(
                scoringClaw.close(),
                scoringArm.home()

        ));

        waitForStart();

        if (isStopRequested())return;

        Actions.runBlocking(new ParallelAction(
                park.build(),
                scoringArm.basket(),
                scoringslides.basket()
        ));

        Actions.runBlocking(new SequentialAction(
                scoringClaw.open(),
                new SleepAction(1),
                scoringClaw.close(),
                scoringArm.home(),
                scoringslides.home(),
                safe.build()
        ));

    }
}
