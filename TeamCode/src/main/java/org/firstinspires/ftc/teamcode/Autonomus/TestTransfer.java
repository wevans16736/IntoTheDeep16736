package org.firstinspires.ftc.teamcode.Autonomus;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configuration.Configuration;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import org.firstinspires.ftc.teamcode.Configuration.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalRollRR;

@Config
@Autonomous(name = "1. Test Transfer", group = "Autonomous")
public class TestTransfer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
        HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);
        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);
        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),
                        horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideRetract),
                        horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristTransfer),
                        horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalClose),

                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

       TrajectoryActionBuilder transferSystem = drive.actionBuilder(initialPose)
               //let go of the butter if it is up ontop of the basket
               .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen))
               .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake))
               .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.bottom))
               //grab the butter from the floor
               .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberClose))
               .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen))
               .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake))
               .waitSeconds(.5)
               //retract the slide
               .afterTime(0, horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideRetract))
               .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristTransfer))
               .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake))
               .waitSeconds(1)
               //vertical grabber close
               .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.verticalClose))
               .waitSeconds(.25)
               //horizontal grabber open
               .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberOpen))
               .waitSeconds(.25)
               //butter go to the other side while priming the horizontal slide for other butter
               .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.verticalWristWall))
               .afterTime(0, horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristIntake))
               .afterTime(0, horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideExtend))
               .waitSeconds(.5)
               .afterTime(0, verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen))
               .waitSeconds(.5)
               .afterTime(0, verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake));

       TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
               .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
               .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom));

        TrajectoryActionBuilder test2 = drive.actionBuilder(initialPose)
                .afterTime(0, verticalSlideRR.verticalSlidePosition(Configuration.highBar))
                .afterTime(1, verticalSlideRR.verticalSlidePosition(Configuration.bottom));

        TrajectoryActionBuilder startPosition = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 0));

        TrajectoryActionBuilder transferMove = drive.actionBuilder(initialPose)
                .waitSeconds(.5)
                .strafeTo(new Vector2d(0, -10));

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder chosenTrajectory;
        Actions.runBlocking(new SequentialAction(startPosition.build()));
        chosenTrajectory = startPosition;

        Action ActionTransferMove = chosenTrajectory.endTrajectory().fresh()
                .waitSeconds(.5)
                .strafeTo(new Vector2d(0, -20))
                .build();

        chosenTrajectory = transferMove;

        Action ActionTransferMoveOther = chosenTrajectory.endTrajectory().fresh()
                .waitSeconds(.5)
                .strafeTo(new Vector2d(0, 0))
                .build();

        Actions.runBlocking(new SequentialAction(new ParallelAction(test.build(), ActionTransferMove), new SleepAction(2), new ParallelAction(ActionTransferMoveOther, test2.build()), new SleepAction(2)));
//        Actions.runBlocking(new SequentialAction(ActionTransferMove, ActionTransferMoveOther));
    }
}
