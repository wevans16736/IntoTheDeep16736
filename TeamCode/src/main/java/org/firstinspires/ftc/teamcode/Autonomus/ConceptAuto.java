package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
@Autonomous(name = "1. Concept Auto", group = "Autonomous")
public class ConceptAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException  {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
        HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);

        VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap, telemetry);
        HorizontalGrabberRR horizontalIntakeRR = new HorizontalGrabberRR(hardwareMap, telemetry);

        VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap, telemetry);
        HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap, telemetry);

        HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap, telemetry);

        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        class RobotSpecial {
            class TransferSystem implements Action{
                boolean primeHorizontal = false;
                public TransferSystem(){
                    primeHorizontal = false;
                }
                public TransferSystem(boolean requestPrime){
                    primeHorizontal = requestPrime;
                }

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if(primeHorizontal) {
                        Actions.runBlocking(new SequentialAction(
                                //grab the butter from the floor
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberClose),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                                new SleepAction(.5),
                                //retract the slide
                                horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideRetract),
                                horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristTransfer),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                                new SleepAction(1),
                                //vertical grabber close
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalClose),
                                new SleepAction(.25),
                                //horizontal grabber open
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberOpen),
                                new SleepAction(.25),
                                //butter go to the other side while priming the horizontal slide for other butter
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristWall),
                                horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristIntake),
                                horizontalSlideRR.horizontalSlidePosition(Configuration.extend),
                                new SleepAction(1.5),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen),
                                new SleepAction(.5),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake)
                        ));
                    }
                    if(!primeHorizontal){
                        Actions.runBlocking(new SequentialAction(
                                //grab the butter from the floor
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberClose),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                                new SleepAction(.5),
                                //retract the slide
                                horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideRetract),
                                horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristTransfer),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                                new SleepAction(1),
                                //vertical grabber close
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalClose),
                                new SleepAction(.25),
                                //horizontal grabber open
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberOpen),
                                new SleepAction(.25),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristWall),
                                new SleepAction(1),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalOpen),
                                new SleepAction(.25),
                                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake)
                        ));
                    }
                    return false;
                }
            }
            public Action transferSystem(boolean primeHorizontal) {return new TransferSystem(primeHorizontal);}
            public Action transferSystem() {return new TransferSystem();}
        }

        TrajectoryActionBuilder test = drive.actionBuilder(currentPose)
                .strafeTo(new Vector2d(0, 20));

        RobotSpecial robotSpecial = new RobotSpecial();



        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.horizontalSlideRetract),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),

                horizontalIntakeRR.horizontalIntakePosition(Configuration.horizontalGrabberOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.verticalClose),

                horizontalWristRR.horizontalWristPosition(Configuration.horizontalWristTransfer),
                verticalWristRR.verticalWristPosition(Configuration.verticalWristIntake),
                horizontalRollRR.horizontalRollPosition(Configuration.flat)
        ));

        TrajectoryActionBuilder startPosition = drive.actionBuilder(currentPose);


        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        TrajectoryActionBuilder chosenTrajectory = startPosition;
        Action ActionTest = chosenTrajectory.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 20))
                .build();

        chosenTrajectory = test;
        Actions.runBlocking(new SequentialAction(
                ActionTest
        ));

        Action ActionSecondTest = chosenTrajectory.endTrajectory().fresh()
                .waitSeconds(3)
                .build();

        Actions.runBlocking(new SequentialAction(ActionSecondTest));
    }
}