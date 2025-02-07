package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="One Specimen")
public class OneSpecimen extends LinearOpMode {

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

        public class Bar_score implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm_left.setPosition(Constants.SCORING_ARM_PICKUP);
                arm_right.setPosition(1 - Constants.SCORING_ARM_PICKUP);
                return false;
            }
        }
        public Action bar_score() {
            return new Bar_score();
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
                boolean done = controller.atSetPoint();

                if (done) {
                    power = 0;
                }

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
                controller.setSetPoint(Constants.SLIDE_BAR+300);
                double power = controller.calculate(leftSlide.getCurrentPosition());
                boolean done = controller.atSetPoint();

                if (done) {
                    power = 0;
                }


                leftSlide.setPower(power);
                rightSlide.setPower(power);

                return !controller.atSetPoint();
            }
        }
        public Action bar() {
            return new Bar();
        }

        public class LowBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                controller.setSetPoint(Constants.SLIDE_BAR-300);
                double power = controller.calculate(leftSlide.getCurrentPosition());
                boolean done = controller.atSetPoint();

                if (done) {
                    power = 0;
                }


                leftSlide.setPower(power);
                rightSlide.setPower(power);

                return !controller.atSetPoint();
            }
        }
        public Action lowBar() {
            return new LowBar();
        }
    }

    public class IntakeSlider{
        private Servo intake_slider;
        public IntakeSlider(HardwareMap hardwareMap){
            intake_slider = hardwareMap.get(Servo.class, "intake_slider");
        }

        public class intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_slider.setPosition(Constants.SLIDER_INTAKE);
                return false;
            }
        }

        public Action Intake() {
            return new intake();
        }

        public class Home implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_slider.setPosition(Constants.SLIDER_HOME);
                return false;
            }
        }
        public Action home() {
            return new Home();
        }


        public class TRANSFER implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_slider.setPosition(Constants.SLIDER_TRANSFER);
                return false;
            }
        }

        public Action Transfer() {
            return new TRANSFER();
        }

    }

    public class IntakeClaw{
        private Servo intake_claw;
        public IntakeClaw(HardwareMap hardwareMap){
            intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_claw.setPosition(Constants.INTAKE_OPEN);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_claw.setPosition(Constants.INTAKE_CLOSE);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }

    public class IntakeArm {
        private Servo intake_arm_rotation_right;
        private Servo intake_arm_rotation_left;

        public IntakeArm(HardwareMap hardwareMap){
            intake_arm_rotation_right = hardwareMap.get(Servo.class, "intake_arm_rotation_r");
            intake_arm_rotation_left = hardwareMap.get(Servo.class, "intake_arm_rotation_l");
        }

        public class Home implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_arm_rotation_left.setPosition(Constants.INTAKE_ROTATION_HOME);
                intake_arm_rotation_right.setPosition(1 - Constants.INTAKE_ROTATION_HOME);
                return false;
            }
        }
        public Action home() {
            return new Home();
        }

        public class Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_arm_rotation_left.setPosition(Constants.INTAKE_CLAW_ROTATION_INTAKE);
                intake_arm_rotation_right.setPosition(1 - Constants.INTAKE_CLAW_ROTATION_INTAKE);
                return false;
            }
        }
        public Action grab() {
            return new Grab();
        }
    }

    public class IntakeClawRotation{
        private Servo intake_claw_rotation;
        public IntakeClawRotation(HardwareMap hardwareMap){
            intake_claw_rotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        }

        public class intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_INTAKE);
                return false;
            }
        }

        public Action Intake() {
            return new IntakeClawRotation.intake();
        }

        public class Home implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_HOME);
                return false;
            }
        }
        public Action home() {
            return new IntakeClawRotation.Home();
        }


        public class TRANSFER implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_TRANSFER);
                return false;
            }
        }

        public Action Transfer() {
            return new IntakeClawRotation.TRANSFER();
        }
    }

    @Override
    public void runOpMode(){
        /* X+ is to the right
         *  Y+ is away from you
         *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(0, -63.5, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);
        ScoringArm scoringArm = new ScoringArm(hardwareMap);
        ScoringClaw scoringClaw = new ScoringClaw(hardwareMap);
        ScoringSlides scoringslides = new ScoringSlides(hardwareMap);
        IntakeSlider intakeSlider = new IntakeSlider(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        IntakeClawRotation intakeClawRotation = new IntakeClawRotation(hardwareMap);



        TrajectoryActionBuilder drive_to_bar = drive.actionBuilder(startpose)
                .strafeTo(new Vector2d(0, -38.5))
                .waitSeconds(2);

        TrajectoryActionBuilder park = drive_to_bar.endTrajectory().fresh()
//                .strafeTo(new Vector2d(55, -63));
                .strafeTo(new Vector2d(0,-63));

        TrajectoryActionBuilder first_block = park.endTrajectory().fresh()
                .strafeTo(new Vector2d(-48, -50))
                .turnTo(Math.toRadians(90))
                .waitSeconds(2);

        Actions.runBlocking(new ParallelAction(
                scoringClaw.close(),
                scoringArm.home(),
                intakeSlider.home(),
                intakeClaw.close(),
                intakeArm.home(),
                intakeClawRotation.home()
        ));

        waitForStart();
        if (isStopRequested())return;

        Actions.runBlocking(new ParallelAction(
                drive_to_bar.build(),
                scoringslides.lowBar(),
                scoringArm.bar_score()
        ));

        Actions.runBlocking(new SequentialAction(
                scoringslides.bar(),
                new SleepAction(1),
                park.build()
        ));

        Actions.runBlocking(new SequentialAction(
                first_block.build(),
                intakeArm.grab(),
                intakeClawRotation.Intake(),
                intakeSlider.Intake(),
                intakeClaw.open(),
                new SleepAction(.5),
                intakeClaw.close()



        ));



    }
}
