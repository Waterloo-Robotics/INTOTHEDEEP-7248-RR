/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.configuration.EditPortListSpinnerActivity;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="7248 Tele", group="Robot")
public class SeaOfElectrons extends OpMode{

    /* Declare OpMode members. */

    public enum IntakeState {
        SUB,
        INTAKE,
        INTAKE_MANUAL,
        TRANSFER,
        HOME,
        TRAVEL
    }

    public enum ScoringState {
        HOME,
        WALL_INTAKE,
        SPECIMEN_SCORE,
        SAMPLE_SCORE,
        SPECIMEN_MANUAL, TRANSFER
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    IntakeState intakeState = IntakeState.HOME;
    ClawState intakeClawState = ClawState.CLOSED;

    ScoringState  scoringState = ScoringState.HOME;
    ClawState scoringClawState = ClawState.CLOSED;

    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftBackDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotorEx rightSlide  = null;
    public DcMotorEx leftSlide  = null;

    IMU imu = null;

    public Servo intake_claw = null;
    public Servo intake_claw_orientation = null;
    public Servo intake_claw_rotation = null;
    public Servo intake_arm_rotation_right = null;
    public Servo intake_arm_rotation_left = null;
    public Servo intake_slider = null;
    public Servo scoring_arm_right = null;
    public Servo scoring_arm_left = null;
    public Servo scoring_claw = null;

    public boolean isIntakeClawPressed = false;
    public boolean isScoringClawPressed = false;
    public boolean isIntakePressed = false;

    ElapsedTime specimenTime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        // Define and Initialize Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bldr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brdr");

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
//
        rightSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "R_slide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "L_slide");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        boolean isRightInitialized = false;
//        boolean isLeftInitialized = false;
//        while (!isRightInitialized || !isLeftInitialized) {
//
//            if (!isLeftInitialized && leftSlide.getCurrent(CurrentUnit.AMPS) < Constants.SLIDE_STALL_THRESH) {
//                leftSlide.setPower(-0.75);
//            } else {
//                leftSlide.setPower(0);
//                isLeftInitialized = true;
//            }
//
//            if (!isRightInitialized && rightSlide.getCurrent(CurrentUnit.AMPS) < Constants.SLIDE_STALL_THRESH) {
//                rightSlide.setPower(-0.75);
//            } else {
//                rightSlide.setPower(0);
//                isRightInitialized = true;
//            }
//
//            telemetry.addData("Right Current", rightSlide.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Current", leftSlide.getCurrent(CurrentUnit.AMPS));
//            telemetry.update();
//        }
//        telemetry.clearAll();
//        telemetry.update();
        /** This needs to be disabled if we have an auto. */
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setTargetPosition(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        intake_claw_orientation = hardwareMap.get(Servo.class, "intake_claw_orientation");
        intake_claw_rotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        intake_arm_rotation_right = hardwareMap.get(Servo.class, "intake_arm_rotation_r");
        intake_arm_rotation_left = hardwareMap.get(Servo.class, "intake_arm_rotation_l");
        intake_slider = hardwareMap.get(Servo.class, "intake_slider");
        scoring_arm_right = hardwareMap.get(Servo.class, "scoring_arm_right");
        scoring_arm_left = hardwareMap.get(Servo.class, "scoring_arm_left");
        scoring_claw = hardwareMap.get(Servo.class, "scoring_claw");

        this.close_intake_claw();
        this.close_scoring_claw();
        this.home_claw_orientation();
        this.straight_claw_rotation();
        this.straight_intake_arm_rotation();
        this.home_intake_slider();
        this.home_scoring_arm();


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        rightSlide.setPower(0.75);
        leftSlide.setPower(0.75);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double forward;
        double strafe;
        double rotation;
        double extend;

        if (gamepad1.left_bumper || gamepad2.left_bumper) {

            if (!isScoringClawPressed) {

                if (scoringClawState == ClawState.OPEN) {
                    scoringClawState = ClawState.CLOSED;
                } else {
                    scoringClawState = ClawState.OPEN;
                }

            }

            isScoringClawPressed = true;

        } else {
            isScoringClawPressed = false;
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {

            if (!isIntakeClawPressed) {

                if (intakeClawState == ClawState.OPEN) {
                    intakeClawState = ClawState.CLOSED;
                } else {
                    if (intakeState == IntakeState.TRANSFER && scoringClawState == ClawState.OPEN) {

                    } else {
                        intakeClawState = ClawState.OPEN;
                    }
                }

            }

            isIntakeClawPressed = true;

        } else {
            isIntakeClawPressed = false;
        }

        if (gamepad1.x || gamepad2.x) {

            if (!isIntakePressed) {

                if (intakeState == IntakeState.SUB) {
                    intakeState = IntakeState.INTAKE;
                    intakeClawState = ClawState.OPEN;
                } else {
                    intakeState = IntakeState.SUB;
                }
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                scoringState = ScoringState.HOME;
                scoringClawState = ClawState.CLOSED;
                rightSlide.setPower(0.75);
                leftSlide.setPower(0.75);

            }

            isIntakePressed = true;

        } else {
            isIntakePressed = false;
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoringState = ScoringState.SAMPLE_SCORE;
            scoringClawState = ClawState.CLOSED;
            intakeState  = IntakeState.HOME;
            rightSlide.setPower(0.75);
            leftSlide.setPower(0.75);
        }

        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoringState = ScoringState.SPECIMEN_SCORE;
            specimenTime.reset();
            scoringClawState = ClawState.CLOSED;
            intakeState  = IntakeState.HOME;
            rightSlide.setPower(0.75);
            leftSlide.setPower(0.75);
        }

        if (gamepad2.dpad_left || gamepad1.dpad_left) {
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            scoringClawState = ClawState.OPEN;

            intakeState = IntakeState.TRANSFER;
            intakeClawState = ClawState.CLOSED;
            scoringState = ScoringState.TRANSFER;
            rightSlide.setPower(0.75);
            leftSlide.setPower(0.75);
        }
        if (gamepad1.left_stick_button || gamepad2.left_stick_button) {
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeState = IntakeState.HOME;
            scoringState = ScoringState.HOME;
            scoringClawState = ClawState.CLOSED;
            rightSlide.setPower(0.75);
            leftSlide.setPower(0.75);
        }
        if (gamepad2.y || gamepad1.y) {
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeState = IntakeState.HOME;
            scoringState = ScoringState.WALL_INTAKE;
            scoringClawState = ClawState.CLOSED;
            rightSlide.setPower(0.75);
            leftSlide.setPower(0.75);
        }

        this.updateIntake();
        this.updateScoring();
        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        forward = gamepad1.left_stick_y * 0.6;
        strafe = -gamepad1.left_stick_x * 0.6;
        rotation = -gamepad1.right_stick_x * 0.6;
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
//        leftFrontDrive.setPower(forward + strafe + rotation);
//        leftBackDrive.setPower(forward - strafe + rotation);
//        rightFrontDrive.setPower(forward - strafe - rotation);
//        rightBackDrive.setPower(forward + strafe - rotation);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.addData("left slide position", leftSlide.getCurrentPosition());
        telemetry.addData("right slide position", rightSlide.getCurrentPosition());
        telemetry.addData("left slide power", leftSlide.getPower());
        telemetry.addData("right slide power", rightSlide.getPower());
        telemetry.addData("left slide current", leftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right slide current", rightSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

    }

    public void updateScoring() {

        if (scoringClawState == ClawState.OPEN) {
            scoring_claw.setPosition(Constants.SCORING_OPEN);
        } else {
            scoring_claw.setPosition(Constants.SCORING_CLOSE);
        }

        if (rightSlide.getCurrent(CurrentUnit.AMPS) > 4.0 && scoringState == ScoringState.HOME && rightSlide.getCurrentPosition() < 100) {

            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setPower(0);

        }

        if (leftSlide.getCurrent(CurrentUnit.AMPS) > 4.0 && scoringState == ScoringState.HOME && leftSlide.getCurrentPosition() < 100) {

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(0);

        }

        switch (scoringState) {

            case HOME:
                leftSlide.setTargetPosition(Constants.SLIDE_HOME);
                rightSlide.setTargetPosition(Constants.SLIDE_HOME);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_HOME);
                scoring_arm_right.setPosition(1 - Constants. SCORING_ARM_HOME);
                break;

            case TRANSFER:
                leftSlide.setTargetPosition(Constants.SLIDE_HOME);
                rightSlide.setTargetPosition(Constants.SLIDE_HOME);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_TRANSFER);
                scoring_arm_right.setPosition(1 - Constants. SCORING_ARM_TRANSFER);
                break;

            case WALL_INTAKE:
                leftSlide.setTargetPosition(Constants.SLIDE_HOME);
                rightSlide.setTargetPosition(Constants.SLIDE_HOME);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_PICKUP);
                scoring_arm_right.setPosition(1 - Constants.SCORING_ARM_PICKUP);
                break;

            case SAMPLE_SCORE:
                leftSlide.setTargetPosition(Constants.SLIDE_BASKET);
                rightSlide.setTargetPosition(Constants.SLIDE_BASKET);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_BASKET);
                scoring_arm_right.setPosition(1 - Constants.SCORING_ARM_BASKET);
                break;

            case SPECIMEN_SCORE:
                leftSlide.setTargetPosition(Constants.SLIDE_BAR);
                rightSlide.setTargetPosition(Constants.SLIDE_BAR);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_PICKUP);
                scoring_arm_right.setPosition(1 - Constants.SCORING_ARM_PICKUP);
                if (leftSlide.getCurrentPosition() > 1370) {
                    scoringState = ScoringState.SPECIMEN_MANUAL;
                }
                break;

            case SPECIMEN_MANUAL:
                double currentPosition = rightSlide.getTargetPosition();
                currentPosition -= gamepad2.left_stick_y * 50;
                rightSlide.setTargetPosition((int) currentPosition);
                leftSlide.setTargetPosition((int) currentPosition);
                scoring_arm_left.setPosition(Constants.SCORING_ARM_PICKUP);
                scoring_arm_right.setPosition(1 - Constants.SCORING_ARM_PICKUP);
                break;

        }

    }

    public void updateIntake() {

        double intake_claw_orientation_position = intake_claw_orientation.getPosition();

        if (intakeClawState == ClawState.OPEN) {
            intake_claw.setPosition(Constants.INTAKE_OPEN);
        } else {
            intake_claw.setPosition(Constants.INTAKE_CLOSE);
        }

        switch (intakeState) {

            case HOME:
                intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_HOME);
                intake_slider.setPosition(Constants.SLIDER_HOME);
                intake_arm_rotation_right.setPosition(Constants.INTAKE_ROTATION_HOME);
                intake_arm_rotation_left.setPosition(1-Constants.INTAKE_ROTATION_HOME);
                break;

            case SUB:
                intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_HOME);
                intake_slider.setPosition(Constants.SLIDER_INTAKE);
                intake_arm_rotation_right.setPosition(Constants.INTAKE_ROTATION_SUB);
                intake_arm_rotation_left.setPosition(1-Constants.INTAKE_ROTATION_SUB);
                break;

            case INTAKE:
                intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_INTAKE);
                intake_slider.setPosition(Constants.SLIDER_INTAKE);
                intake_arm_rotation_right.setPosition(Constants.INTAKE_ROTATION_INTAKE);
                intake_arm_rotation_left.setPosition(1-Constants.INTAKE_ROTATION_INTAKE);
                intakeState = IntakeState.INTAKE_MANUAL;
                break;

            case INTAKE_MANUAL:
                intake_claw_orientation_position -= gamepad1.right_trigger*0.01;
                intake_claw_orientation_position += gamepad1.left_trigger*0.01;
                intake_claw_orientation_position -= gamepad2.right_trigger*0.01;
                intake_claw_orientation_position += gamepad2.left_trigger*0.01;
                intake_claw_orientation.setPosition(intake_claw_orientation_position);
                break;

            case TRANSFER:
                intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
                intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_TRANSFER);
                intake_slider.setPosition(Constants.SLIDER_TRANSFER);
                intake_arm_rotation_right.setPosition(Constants.INTAKE_ROTATION_HOME);
                intake_arm_rotation_left.setPosition(1-Constants.INTAKE_ROTATION_HOME);
                break;

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void open_intake_claw() {
        intake_claw.setPosition(Constants.INTAKE_OPEN);
    }


    public void close_intake_claw() {
        intake_claw.setPosition(Constants.INTAKE_CLOSE);
    }


    public void open_scoring_claw() {
        scoring_claw.setPosition(Constants.SCORING_OPEN);
    }


    public void close_scoring_claw() {
        scoring_claw.setPosition(Constants.SCORING_CLOSE);
    }

    public void home_claw_orientation() {
        intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
    }


    public void straight_claw_rotation() {
        intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_HOME);
    }

    public void straight_intake_arm_rotation() {
        double position = Constants.INTAKE_ROTATION_HOME;
        intake_arm_rotation_right.setPosition(position);
        intake_arm_rotation_left.setPosition(1-position);
    }
    public void home_intake_slider() {
        intake_slider.setPosition(Constants.SLIDER_HOME);
    }

    public void home_scoring_arm() {
        double position = Constants.SCORING_ARM_HOME;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void transfer_scoring_arm() {
        double position = Constants.SCORING_ARM_TRANSFER;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void score_scoring_arm() {
        double position = Constants.SCORING_ARM_PICKUP;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void sub_intake_arm_rotation() {
        double position = Constants.INTAKE_ROTATION_SUB;
        intake_arm_rotation_right.setPosition(position);
        intake_arm_rotation_left.setPosition(1-position);
    }

    public void intake_claw_rotation_intake() {
        intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_INTAKE);
    }

    public void intake_claw_arm_rotation (){
        double position = Constants.INTAKE_ROTATION_INTAKE;
        intake_arm_rotation_right.setPosition(position);
        intake_arm_rotation_left.setPosition(1-position);
    }

    public void intake_slider_intake(){
        intake_slider.setPosition(Constants.SLIDER_INTAKE);
    }

}
