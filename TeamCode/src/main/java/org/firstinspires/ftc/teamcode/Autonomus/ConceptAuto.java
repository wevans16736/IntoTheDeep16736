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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;


import org.firstinspires.ftc.teamcode.Configuration.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.HorizontalRollRR;

@Config
@Autonomous(name = "1. Final Auto", group = "Autonomous")
public class ConceptAuto extends LinearOpMode {
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

    RobotSpecial robotSpecial = new RobotSpecial();
    public class RobotSpecial {
            public class TransferSystem implements Action{
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
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                                new SleepAction(.5),
                                //retract the slide
                                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                                new SleepAction(1),
                                //vertical grabber close
                                verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                                new SleepAction(.25),
                                //horizontal grabber open
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                                new SleepAction(.25),
                                //butter go to the other side while priming the horizontal slide for other butter
                                verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                                horizontalWristRR.horizontalWristPosition(Configuration.forwardPosOut),
                                horizontalSlideRR.horizontalSlidePosition(Configuration.extend),
                                new SleepAction(1.5),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                                new SleepAction(.5),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                        ));
                    }
                    if(!primeHorizontal){
                        Actions.runBlocking(new SequentialAction(
                                //grab the butter from the floor
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorClose),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                                new SleepAction(.5),
                                //retract the slide
                                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
                                new SleepAction(1),
                                //vertical grabber close
                                verticalGrabberRR.verticalGrabberPosition(Configuration.close),
                                new SleepAction(.25),
                                //horizontal grabber open
                                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                                new SleepAction(.25),
                                verticalWristRR.verticalWristPosition(Configuration.forwardDown),
                                new SleepAction(1),
                                verticalGrabberRR.verticalGrabberPosition(Configuration.open),
                                new SleepAction(.25),
                                verticalWristRR.verticalWristPosition(Configuration.backwardPos)
                        ));
                    }
                    return false;
                }
            }
            public Action transferSystem(boolean primeHorizontal) {return new TransferSystem(primeHorizontal);}
            public Action transferSystem() {return new TransferSystem();}
        public class SelfCorrect implements Action {
            Pose2d currentPose = drive.getLastPinpointPose();
            Pose2d currentError = MecanumDrive.currentError;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                dashboardTelemetry.addData("X", currentPose.position.x);
                dashboardTelemetry.addData("Y", currentPose.position.y);
                dashboardTelemetry.addData("Heading", currentPose.heading.toDouble());

                dashboardTelemetry.addData("X error", currentError.position.x);
                dashboardTelemetry.addData("Y Error", currentError.position.y);
                dashboardTelemetry.addData("Heading Error", currentError.heading.toDouble());
                dashboardTelemetry.update();
                TrajectoryActionBuilder correctError = drive.actionBuilder(currentPose)
                        .strafeToLinearHeading(new Vector2d(currentPose.position.x - currentError.position.x,
                                        currentPose.position.y - currentError.position.y),
                                currentPose.heading.toDouble() - currentError.heading.toDouble());
                Actions.runBlocking(new SequentialAction(
                        correctError.build()
                ));
                return false;
            }
        }
        public Action selfCorrect() {return new RobotSpecial.SelfCorrect();}
    }

    TrajectoryActionBuilder test = drive.actionBuilder(currentPose)
            .strafeTo(new Vector2d(0, 20));

    @Override
    public void runOpMode() throws InterruptedException  {
        //initialize the robot before starting
        Actions.runBlocking(new SequentialAction(
                horizontalSlideRR.horizontalSlidePosition(Configuration.retractSlide),
                verticalSlideRR.verticalSlidePosition(Configuration.bottom),

                horizontalIntakeRR.horizontalIntakePosition(Configuration.floorOpen),
                verticalGrabberRR.verticalGrabberPosition(Configuration.close),

                horizontalWristRR.horizontalWristPosition(Configuration.backwardPosIn),
                verticalWristRR.verticalWristPosition(Configuration.backwardPos),
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
                .afterTime(0, robotSpecial.selfCorrect())
                .build();

        Actions.runBlocking(new SequentialAction(ActionSecondTest));
    }
}