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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftBackDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor  arm = null;
    TouchSensor HangerBase = null;
    TouchSensor HangerTop = null;

    public boolean arm_located = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bldr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brdr");
        arm = hardwareMap.get(DcMotor.class, "arm");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HangerBase = hardwareMap.get(TouchSensor.class, "HBLimit");
        HangerTop = hardwareMap.get(TouchSensor.class, "HTLimit");

        arm_located = false;

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

        if (arm_located) {
            if (gamepad2.dpad_up){
                arm.setTargetPosition(-10147);
            } else if (gamepad2.dpad_down) {
                arm.setTargetPosition(0);
            }

            if ((HangerBase.isPressed() && arm.getTargetPosition() == 0) ||
                    (HangerTop.isPressed() && arm.getTargetPosition() == -10147))
            {
                arm.setPower(0);
            } else  {
                arm.setPower(1);
            }

        } else {
            arm.setPower(0.5);
            if (HangerBase.isPressed()) {
                arm_located = true;
                arm.setPower(0);
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }


        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotation = -gamepad1.right_stick_x;

        leftFrontDrive.setPower(forward + strafe + rotation);
        leftBackDrive.setPower(forward - strafe + rotation);
        rightFrontDrive.setPower(forward - strafe - rotation);
        rightBackDrive.setPower(forward + strafe - rotation);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.addData("arm", "%b", arm_located);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
