package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.botsquadutil.PID;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="AutoLeft", group="2022-2023")
public class AutoLeft extends LinearOpMode {

    BNO055IMU imu;
    DcMotor TL, TR, BL, BR, ARB1, ARB2;
    List<DcMotor> driveMotors;
    List<DcMotor> ARM;
    Servo claw;
    double proportional = 1.0;
    double robPosX = 0.0, robPosY = 0.0;
    double lastTime = 0.0;

    double ARBWheelRad = 0.0;
    double ARBWheelCirc = 2.0 * Math.PI * ARBWheelRad;
    int ARBTickPerRev = 28;

    ElapsedTime runTime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    private PID angularPID = new PID(0.36, 0.15, 0.045);
    private PID linearPID = new PID(0.2, 0.0, 0.0);

    //New Configs for C270
    //Units are in pixels
    double fx = 1996.17;
    double fy = 1996.17;
    double cx = 319.5;
    double cy = 239.5;
    double TRPower, TLPower, BLPower, BRPower;

    // UNITS ARE METERS
    double tagsize = 0.508;

    //Tag IDs of Sleeve
    int id_Left = 5;
    int id_Mid = 13;
    int id_Right = 18;

    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;

    @Override
    public void runOpMode() {
        //Initialize Hardware
        initHardware();

        //Wait for start and detect the sleeve
        detectWhileWaitForStart();


        telemetry.addData("tagFound: ", tagFound);
        telemetry.update();

        runTime.reset();
        if (opModeIsActive()) {
            if (tagFound) {
                if (tagOfInterest.id == id_Left) {
                    runTo(0.0, 1.0, 0.0, 0.15);
                    sleep(500);
                    runTo(-1.0, 0.0, 0.0, 1.95);
                    sleep(500);
                    runTo(0.0, 1.0, 0.0, 1.8);

                } else if (tagOfInterest.id == id_Mid) {
                    runTo(0.0, 1.0, 0.0, 1.8);

                } else if (tagOfInterest.id == id_Right) {
                    runTo(0.0, 1.0, 0.0, 0.15);
                    sleep(500);
                    runTo(1.0, 0.0, 0.0, 1.65);
                    sleep(500);
                    runTo(0.0, 1.0, 0.0, 1.8);
                }
                //default path
                else {

                }
            } else {

            }
        }
    }

    //Units: inches
    private void armGoTo(double height){
        int ticks = (int)Math.round(height / ARBWheelCirc) * ARBTickPerRev;

        ARB1.setTargetPosition(ticks);
        ARB2.setTargetPosition(ticks);
    }

    private void initHardware(){
        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        ARB1 = hardwareMap.get(DcMotor.class, "ARB1");
        ARB2 = hardwareMap.get(DcMotor.class, "ARB2");

        claw = hardwareMap.get(Servo.class, "claw");

        driveMotors = Arrays.asList(TL, TR, BL, BR);
        ARM = Arrays.asList(ARB1, ARB2);

        TR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB1.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB2.setDirection(DcMotorSimple.Direction.FORWARD);

        setRunToPosition(ARM, 0.25);
        setMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehaviour(driveMotors, DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Initializing AprilTagDetection (Detection of the sleeve)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Originally 800 and 448
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    //Detects the sleeve while waiting for the start
    private void detectWhileWaitForStart(){
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == id_Left || tag.id == id_Mid || tag.id == id_Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
    }

    public void setMode(List<DcMotor> motorList, DcMotor.RunMode runmode){
        for(DcMotor motor : motorList){
            motor.setMode(runmode);
        }
    }

    public void setZeroPowerBehaviour(List<DcMotor> motorList, DcMotor.ZeroPowerBehavior behaviour){
        for(DcMotor motor : motorList){
            motor.setZeroPowerBehavior(behaviour);
        }
    }

    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    //Robot-centric
    //Use meters
    public void goTo(double x, double y, double heading, double motorScalingFactor, double minErr){
        double errX = x - imu.getPosition().x;
        double errY = y - imu.getPosition().y;

        while(Math.abs(errX) > minErr && Math.abs(errY) > minErr) {
            telemetry.addData("tagFound", tagFound);

            double deltaTime = getDelTime();

            PID.tunedOut tndOut = checkStrafeAndHead(x, y, heading, deltaTime);

            double tunedAngle = tndOut.angle;
            double tunedX = tndOut.x;
            double tunedY = tndOut.y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(tunedAngle) + Math.abs(tunedY) + Math.abs(tunedX), 1);
            double leftPow = ((y + tunedY) - (x + tunedX)) * motorScalingFactor;
            double rightPow = ((y + tunedY) + (x + tunedX)) * motorScalingFactor;

            TRPower = (rightPow - tunedAngle) / denominator;
            BLPower = (leftPow - tunedAngle) / denominator;
            TLPower = (leftPow + tunedAngle) / denominator;
            BRPower = (rightPow + tunedAngle) / denominator;

            TR.setPower(TRPower);
            BL.setPower(BLPower);
            TL.setPower(TLPower);
            BR.setPower(BRPower);

            telemetry.addLine("We're here in the PID section");

            telemetry.addData("TRPow", TRPower);
            telemetry.addData("TLPow", TLPower);
            telemetry.addData("BRPow", BRPower);
            telemetry.addData("BLPow", BLPower);

            telemetry.addData("ErrX", errX);
            telemetry.addData("ErrY", errY);
            telemetry.update();

            errX = x - imu.getPosition().x;
            errY = y - imu.getPosition().y;
        }
    }

    public void runTo(double x, double y, double head, double time){
        double execTime = runTime.time();
        double startTime = runTime.time();

        y = -y;
        while (execTime < startTime + time) {
            telemetry.addData("Runtime: ", runTime.time());
            telemetry.update();

            double chassisSpeedMultiplier = 0.10;

            double frameTime = runTime.time();
            double delTime = frameTime - execTime;
            execTime = frameTime;

            double angleError = checkAngle(x, y, head, delTime);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(angleError), 1);
            TRPower = ((y + x) * chassisSpeedMultiplier - angleError) / denominator;
            BLPower = ((y - x) * chassisSpeedMultiplier - angleError) / denominator;
            TLPower = ((y - x) * chassisSpeedMultiplier + angleError) / denominator;
            BRPower = ((y + x) * chassisSpeedMultiplier + angleError) / denominator;

            TL.setPower(TLPower);
            BL.setPower(BLPower);
            TR.setPower(TRPower);
            BR.setPower(BRPower);
        }

        TL.setPower(0.0);
        BL.setPower(0.0);
        TR.setPower(0.0);
        BR.setPower(0.0);
    }

    public void rotTo(double angle, double motorScalingValue, double minErr){
        double angleErr = (angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        while(Math.abs(angleErr) > minErr){
            double deltaTime = getDelTime();

            PID.tunedOut tndOut = checkStrafeAndHead(0.0, 0.0, angle, deltaTime);

            double tunedAngle = tndOut.angle;

            TR.setPower(-tunedAngle);
            BL.setPower(-tunedAngle);
            TL.setPower(tunedAngle);
            TR.setPower(tunedAngle);

            angleErr = (angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        }
    }

    public double getDelTime(){
        double currentTime = runTime.time();
        lastTime = currentTime;

        return currentTime - lastTime;
    }

    public PID.tunedOut checkStrafeAndHead(double expectedX, double expectedY, double expectedHeading, double delTime){
        double tunedX = linearPID.output(imu.getPosition().x, expectedX, delTime);
        double tunedY = linearPID.output(imu.getPosition().y, expectedY, delTime);
        double tunedAngle = angularPID.output(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, expectedHeading, delTime);

        PID.tunedOut output = new PID.tunedOut(tunedX, tunedY, tunedAngle);

        return output;
    }

    public double checkAngle(double expectedX, double expectedY, double expectedHeading, double delTime){
        double robHead = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;;
        double angleErr = expectedHeading - robHead;

        return angularPID.output(robHead, expectedHeading, delTime);
    }

    //sets all motors in a list of motors to mode "Run To Position"
    private void setRunToPosition(List<DcMotor> motors, double normalizedPow){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(normalizedPow);
        }
    }
}
