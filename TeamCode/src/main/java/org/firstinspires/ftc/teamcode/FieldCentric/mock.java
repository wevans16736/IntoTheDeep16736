package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp(name = "bot")
public class mock extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d currentPose = new Pose2d(9,-64,Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);
        boolean stuff = true;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if(stuff) {
                Actions.runBlocking(
                drive.actionBuilder(currentPose)
                        .strafeToConstantHeading(new Vector2d(0, -64))
                        .build()
                );
                stuff = false;
            }
        }
    }
}
