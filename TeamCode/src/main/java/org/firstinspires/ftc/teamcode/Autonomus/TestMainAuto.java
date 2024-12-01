package org.firstinspires.ftc.teamcode.Autonomus;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

import java.util.Arrays;

@Config
@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestMainAuto extends LinearOpMode {
    private VerticalSlideRR verticalSlideRR = null;
    private HorizontalSlideRR horizontalSlideRR = null;
    private VerticalGrabberRR verticalGrabberRR = null;
    private VerticalWristRR verticalWristRR = null;
    private HorizontalWristRR horizontalWristRR = null;
    private HorizontalGrabberRR horizontalGrabberRR = null;
    private HorizontalRollRR horizontalRollRR = null;

    boolean pickRightSide = false;
    boolean pickLeftSide = false;
    boolean pickHang = false;
    boolean pickHuman = false;
    boolean pickBasket = false;
    boolean pickPark = false;
    boolean pickButter = false;

    int hangSide = 1;
    int butterSide = 1;
    int parkSide = 1;
    int waitHuman = 0;

    public class VerticalSlideRR{
        public DcMotorEx verticalSlide1 = null;
        public DcMotorEx verticalSlide2 = null;

        public VerticalSlideRR(HardwareMap hardwareMap){
            verticalSlide1 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setTargetPosition(0);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            verticalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setTargetPosition(0);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public class Liftup implements Action {
            int position = Configuration.highBar;
            private boolean initialized = false;
            public Liftup(int p){
                this.position = p;
            }
            public Liftup(){
                this.position = Configuration.highBar;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double velocity = 1800;
                if (!initialized) {
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                if(currentPosition > position) {
                    return true;
                } else {
                    return false;

                }
            }
        }
        public Action liftUp(int p){
            return new Liftup(p);
        }
        public Action liftUp(){
            return new Liftup();
        }
        public class SetDown implements Action{
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet){
                int position = 0;
                double velocity = 1800;
                if(!initialized){
                    verticalSlide1.setTargetPosition(position);
                    verticalSlide1.setVelocity(velocity);
                    verticalSlide2.setTargetPosition(-position);
                    verticalSlide2.setVelocity(-velocity);
                    initialized = true;
                }
                double currentPosition = verticalSlide1.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action setDown(){
            return new SetDown();
        }
    }
    //make a class for horizontal Slide
    public class HorizontalSlideRR{
        public DcMotorEx HorizontalSlide2 = null;
        private Telemetry telemetry;
        public HorizontalSlideRR(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            HorizontalSlide2 = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_SLIDE2);
            HorizontalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            HorizontalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            HorizontalSlide2.setTargetPosition(0);
            HorizontalSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        private boolean initialized = false;
        public class RetractSlide implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int position = Configuration.retractSlide;
                double velocity = 1800;
                if(!initialized){
                    HorizontalSlide2.setTargetPosition(position);
                    HorizontalSlide2.setVelocity(velocity);
                    initialized = true;
                }
                double currentPosition = HorizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action retractSlide(){
            return new RetractSlide();
        }
        public class ExtendButter implements Action{
            private boolean initialized = false;
            int extend = Configuration.extend;
            double extendVelocity = Configuration.extendVelocity;
            public boolean run(@NonNull TelemetryPacket packet) {
                int position = extend;
                double velocity = extendVelocity;
                if (!initialized) {
                    HorizontalSlide2.setTargetPosition(position);
                    HorizontalSlide2.setVelocity(velocity);
                    initialized = true;
                }
                double currentPosition = HorizontalSlide2.getCurrentPosition();
                return currentPosition > position;
            }
        }
        public Action extendButter(){
            return new ExtendButter();
        }
    }
    //add a class for the vertical grabber
    public class VerticalGrabberRR{
        public Servo verticalGrabberServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private double open = Configuration.open;
        private double close = Configuration.close;
        public VerticalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalGrabberServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
            verticalGrabberServo.setPosition(1.0);
        }
        public class CloseGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(close);
                return false;
            }
        }
        public Action closeGrabber(){
            return new CloseGrabber();
        }
        public class OpenGrabber implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalGrabberServo.setPosition(open);
                return false;
            }
        }
        public Action openGrabber(){
            return new OpenGrabber();
        }
    }
    //add Vertical Wrist Class
    public class VerticalWristRR{
        public Servo verticalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        double forwardUp = Configuration.forwardUp;
        double forwardDown = Configuration.forwardDown;
        double backwardPos = Configuration.backwardPos;
        public VerticalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            verticalWristServo = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_WRIST);
            verticalWristServo.setPosition(backwardPos);
        }
        public class PlaceBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardUp);
                return false;
            }
        }
        public Action placeBasket(){
            return new PlaceBasket();
        }
        public class WallButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(forwardDown);
                return false;
            }
        }
        public Action wallButter(){
            return new WallButter();
        }
        public class TakeButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                verticalWristServo.setPosition(backwardPos);
                return false;
            }
        }
        public Action takeButter(){
            return new TakeButter();
        }
    }
    public class HorizontalWristRR{
        public Servo horizontalWristServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private double backwardPosIn = Configuration.backwardPosIn;
        private double backwardPosOut = Configuration.backwardPosOut;
        private double forwardPosOut = Configuration.forwardPosOut;
        public HorizontalWristRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            horizontalWristServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_WRIST);
            horizontalWristServo.setPosition(backwardPosIn);
        }
        public class GrabButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(forwardPosOut);
                return false;
            }
        }
        public Action grabButter(){
            return new GrabButter();
        }
        public class InRobot implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizontalWristServo.setPosition(backwardPosIn);
                return false;
            }
        }
        public Action inRobot(){
            return new InRobot();
        }
        public class LiftButter implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                horizontalWristServo.setPosition(backwardPosOut);
                return false;
            }
        }
        public Action liftButter(){
            return new LiftButter();
        }
    }
    //HorizontalGrabber class
    public class HorizontalGrabberRR{
        public Servo intakeServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public HorizontalGrabberRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);
            intakeServo.setPosition(0);
        }
        private double floorClose = Configuration.floorClose;
        private double floorOpen = Configuration.floorOpen;
        public class FloorClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorClose);
                return false;
            }
        }
        public Action floorClose(){
            return new FloorClose();
        }
        public class FloorOpen implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServo.setPosition(floorOpen);
                return false;
            }
        }
        public Action floorOpen(){
            return new FloorOpen();
        }
    }
    public class HorizontalRollRR{
        public Servo rollServo;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        public double flat = Configuration.flat;
        public HorizontalRollRR(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
            this.telemetry = opModeTelemetry;
            this.hardwareMap = opModeHardware;
            rollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
            rollServo.setPosition(flat);
        }
        public class Flat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rollServo.setPosition(flat);
                return false;
            }
        }
        public Action flat(){
            return new Flat();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //instantiate the robot to a particular pose.
        verticalSlideRR = new VerticalSlideRR(hardwareMap);
        horizontalSlideRR = new HorizontalSlideRR(hardwareMap, telemetry);
        verticalGrabberRR = new VerticalGrabberRR(telemetry, hardwareMap);
        verticalWristRR = new VerticalWristRR(telemetry, hardwareMap);
        horizontalWristRR = new HorizontalWristRR(telemetry, hardwareMap);
        horizontalGrabberRR = new HorizontalGrabberRR(telemetry, hardwareMap);
        horizontalRollRR = new HorizontalRollRR(telemetry, hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        Vector2d vector2d = new Vector2d(0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        VelConstraint pushBlockVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint pushBlockAccelOverride = new ProfileAccelConstraint(-10, 25);

        VelConstraint parkVelOverride = new TranslationalVelConstraint(80);
        VelConstraint parkAngularOverride = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        AccelConstraint parkAccelOverride = new ProfileAccelConstraint(-50, 50);

        VelConstraint humanVelOverride = new TranslationalVelConstraint(30);
        AccelConstraint humanAccelOverride = new ProfileAccelConstraint(-7, 50);

        TrajectoryActionBuilder test = drive.actionBuilder(drive.getPoseX());

        drive.updatePoseEstimate();

        //initialize the robot
        Actions.runBlocking(
                new SequentialAction(
                        verticalGrabberRR.closeGrabber(),
                        verticalWristRR.takeButter(),
                        horizontalSlideRR.retractSlide(),
                        horizontalWristRR.inRobot(),
                        horizontalGrabberRR.floorClose(),
                        horizontalRollRR.flat()
                )
        );

        //wait for the start button to be press
        waitForStart();
        //if stop button is press, automatically stop
        if (isStopRequested()) return;

        //choose the trajectory
        //todo figure out how to incorporate this
        Action trajectoryActionChosen;
        if (pickLeftSide) {
            if(pickHang) {
//                trajectoryActionChosen = trajectoryActionChosen+ hang.build();
            }

            //run the chosen action blocking
            Actions.runBlocking(
                    new SequentialAction(
                    )
            );
        }
    }
}



