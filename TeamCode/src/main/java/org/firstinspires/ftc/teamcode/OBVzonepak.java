package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OBVzonepak")
public class OBVzonepak extends LinearOpMode {


    @Override
public void runOpMode(){
        /* X+ is to the right
        *  Y+ is away from you
        *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(0, -63.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);

        TrajectoryActionBuilder park = drive.actionBuilder(startpose)
                .strafeTo(new Vector2d(24,-60))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-24,-60));

        waitForStart();

        if (isStopRequested())return;

        Actions.runBlocking(
                park.build()
        );

    }
}
