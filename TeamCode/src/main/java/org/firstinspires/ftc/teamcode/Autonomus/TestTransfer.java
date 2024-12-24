package org.firstinspires.ftc.teamcode.Autonomus;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configuration.ConfigurationFirstRobot;
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

        class RobotSpecial {
            class TransferSystem implements Action {
                boolean primeHorizontal = false;
                boolean pickLeft = false;

                public TransferSystem() {
                    primeHorizontal = false;
                }

                public TransferSystem(boolean pickLeft, boolean requestHorizontal) {
                    primeHorizontal = requestHorizontal;
                    this.pickLeft = pickLeft;
                }

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (pickLeft) {
                        Actions.runBlocking(new SequentialAction(
                                //let go of the butter on top
                                verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                new SleepAction(.5),
                                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.bottom),
                                new SleepAction(2),
                                //extend the horizontal Slide
                                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalSlideExtend),
                                horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake),
                                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
                                new SleepAction(.5),
                                //grab the butter from the floor
                                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose),
                                verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                new SleepAction(.5),
                                //retract the slide
                                horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract),
                                horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer),
                                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                new SleepAction(1),
                                //vertical grabber close
                                verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose),
                                new SleepAction(.25),
                                //horizontal grabber open
                                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
                                new SleepAction(.25),
                                verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.topBasket),
                                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristBasket)
                        ));
                    }
                    if (!pickLeft) {
                        if (primeHorizontal) {
                            Actions.runBlocking(new SequentialAction(
                                    //let go of the butter if it is up ontop of the basket
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                    verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.bottom),
                                    //grab the butter from the floor
                                    horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose),
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                    new SleepAction(.5),
                                    //retract the slide
                                    horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract),
                                    horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                    new SleepAction(1),
                                    //vertical grabber close
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose),
                                    new SleepAction(.25),
                                    //horizontal grabber open
                                    horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
                                    new SleepAction(.25),
                                    //butter go to the other side while priming the horizontal slide for other butter
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristWall),
                                    horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake),
                                    horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideExtend),
                                    new SleepAction(1.5),
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                    new SleepAction(.5),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake)
                            ));
                        }
                        if (!primeHorizontal) {
                            Actions.runBlocking(new SequentialAction(
                                    //grab the butter from the floor
                                    horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose),
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                    new SleepAction(.5),
                                    //retract the slide
                                    horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract),
                                    horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                                    new SleepAction(1),
                                    //vertical grabber close
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose),
                                    new SleepAction(.25),
                                    //horizontal grabber open
                                    horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
                                    new SleepAction(.25),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristWall),
                                    new SleepAction(1),
                                    verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalOpen),
                                    new SleepAction(.25),
                                    verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake)
                            ));
                        }
                    }
                    return false;
                }
            }

            public Action transferSystem(boolean pickLeft, boolean primeHorizontal) {
                return new TransferSystem(pickLeft, primeHorizontal);
            }

            public Action transferSystem() {
                return new TransferSystem();
            }
        }

        RobotSpecial robotSpecial = new RobotSpecial();

        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideRetract),
                verticalSlideRR.verticalSlidePosition(ConfigurationFirstRobot.bottom),

                horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberOpen),
                verticalGrabberRR.verticalGrabberPosition(ConfigurationFirstRobot.verticalClose),

                horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristTransfer),
                verticalWristRR.verticalWristPosition(ConfigurationFirstRobot.verticalWristIntake),
                horizontalRollRR.horizontalRollPosition(ConfigurationFirstRobot.flat)
        ));

        TrajectoryActionBuilder startPosition = drive.actionBuilder(initialPose)
                .afterTime(0, horizontalIntakeRR.horizontalIntakePosition(ConfigurationFirstRobot.horizontalGrabberClose))
                .afterTime(0, horizontalWristRR.horizontalWristPosition(ConfigurationFirstRobot.horizontalWristIntake))
                .afterTime(0, horizontalSlideRR.horizontalSlidePosition(ConfigurationFirstRobot.horizontalSlideExtend))
                .strafeTo(new Vector2d(0, -10))
                .afterDisp(0, robotSpecial.transferSystem(false, true))
                .afterDisp(0, robotSpecial.transferSystem(false, true))
                .afterDisp(0, robotSpecial.transferSystem(false, false))
                        .waitSeconds(2);

        //wait for the start button to be press
        waitForStart();
        //if the stop button press then stop the robot
        if (isStopRequested()) return;

        Actions.runBlocking(startPosition.build());
    }
}
