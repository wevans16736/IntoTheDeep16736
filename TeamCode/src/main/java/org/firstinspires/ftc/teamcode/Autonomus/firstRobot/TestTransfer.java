package org.firstinspires.ftc.teamcode.Autonomus.firstRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "1. Test Transfer", group = "Autonomous")
public class TestTransfer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
//        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
//        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
//        HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);
//        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
//        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);
//        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //initialize the robot before starting
//        Actions.runBlocking(new SequentialAction(
//                horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideExtend),
//                horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake),
//                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
//                verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose),
//
//                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
//                horizontalRollRR.horizontalRollPosition(ConfigurationFirstRobot.flat)
//        ));

//        TrajectoryActionBuilder transferSystem = drive.actionBuilder(initialPose)
//                //let go of the butter if it is up ontop of the basket
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .afterTime(0, verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.bottom))
//                //grab the butter from the floor
//                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose))
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .waitSeconds(.5)
//                //retract the slide
//                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract))
//                .afterTime(0, horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .waitSeconds(1)
//                //vertical grabber close
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose))
//                .waitSeconds(.25)
//                //horizontal grabber open
//                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen))
//                .waitSeconds(.25)
//                //butter go to the other side while priming the horizontal slide for other butter
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristWall))
//                .afterTime(0, horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake))
//                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideExtend))
//                .waitSeconds(.5)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .waitSeconds(.5)
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake));
//
//        TrajectoryActionBuilder transferSystem2 = drive.actionBuilder(initialPose)
//                //let go of the butter if it is up ontop of the basket
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .afterTime(0, verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.bottom))
//                //grab the butter from the floor
//                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose))
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .waitSeconds(.5)
//                //retract the slide
//                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract))
//                .afterTime(0, horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer))
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake))
//                .waitSeconds(1)
//                //vertical grabber close
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose))
//                .waitSeconds(.25)
//                //horizontal grabber open
//                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen))
//                .waitSeconds(.25)
//                //butter go to the other side while priming the horizontal slide for other butter
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristWall))
//                .afterTime(0, horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake))
//                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideExtend))
//                .waitSeconds(.5)
//                .afterTime(0, verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen))
//                .waitSeconds(.5)
//                .afterTime(0, verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake));
//
//        TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.highBar))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(0))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.highBar))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(0))
//                .endTrajectory();
//
//        TrajectoryActionBuilder test2 = drive.actionBuilder(initialPose)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.highBar))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(0))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.highBar))
//                .waitSeconds(1)
//                .stopAndAdd(verticalSlideRR.verticalSlidePosition(0))
//                .endTrajectory();

        TrajectoryActionBuilder startPosition = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 0));

        TrajectoryActionBuilder transferMove = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -40));

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder chosenTrajectory;
        Actions.runBlocking(new SequentialAction(startPosition.build()));
        chosenTrajectory = startPosition;

        Action ActionTransferMove = chosenTrajectory.endTrajectory().fresh()
                .splineTo(new Vector2d(-30, 15), Math.PI / 2)
                .splineTo(new Vector2d(-60, -30), Math.PI)
                .build();

        chosenTrajectory = transferMove;

        Action ActionTransferMoveOther = chosenTrajectory.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();

        Actions.runBlocking(new SequentialAction(ActionTransferMove));
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(transferSystem2.build(), ActionTransferMoveOther),
//                new SleepAction(2)));
    }
}
