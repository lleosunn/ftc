package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class tensorflow extends LinearOpMode {
    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ASK1IFv/////AAABmfX8FNhgmkD+sKsolRwQ6ShIKlMBrgagR3WWfuzHquZIi3lQ5TFj8sAK1gmeujmQm8I62YVaT/Z3X5XIHGawMeaNm7BnVeU5scz+HdlNLlVrCDbuIb8sJ29tn/mBfWuv3Hvy40iP9uOtEAKi9diyxRsAiytHqfsCvccIem7+C7O6Zbiz3awD5CobXCqevYjWdUGGZarPM2eAyL/NfCswBIdYd1Hjb3VHqJ/tyb1VZdWYlme0pkAKLWbYpRKFlq5Q8EOrzmVWTAw4fCLziKSKYFVFFPNoibGm7cAGsqk+5UD3kOpcoPOhSlDM3GEPMYartSro28s1DB1uFaojEVNE+PJ5fMaWWCsfi9japsdcrP6Q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public void imuinit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        telemetry.addData("Gyro Mode", "calibrating...");

        telemetry.update();


        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Gyro Mode", "ready");

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        telemetry.update();

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            // tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor intake = hardwareMap.get(DcMotor.class,"intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        motorblock block = new motorblock(tl, tr, bl, br);
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensor_range2");
        sensorRange3 = hardwareMap.get(DistanceSensor.class, "sensor_range3");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorRange2;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) sensorRange3;

        tl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imuinit();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                stopper.setPosition(1);
                while (sensorRange3.getDistance(DistanceUnit.INCH)<45) { //back
                    double power = 0.3;
                    telemetry.addData("angle", getAngle());
                    telemetry.update();
                    if (getAngle() < -1){ //turn left
                        tl.setPower(-(power - (0.01 * getAngle())));
                        tr.setPower((power + (0.01 * getAngle())));
                        bl.setPower(-(power - (0.01 * getAngle())));
                        br.setPower((power + (0.01 * getAngle())));
                    } else if (getAngle() > 1){ //turn right
                        tl.setPower(-(power - (0.01 * getAngle())));
                        tr.setPower((power + (0.01 * getAngle())));
                        bl.setPower(-(power - (0.01 * getAngle())));
                        br.setPower((power + (0.01 * getAngle())));
                    } else {
                        tl.setPower(-power);
                        tr.setPower(power);
                        bl.setPower(-power);
                        br.setPower(power);
                    }
                }
                block.stop();
                sleep(3000);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) telemetry.addData("label",updatedRecognitions.get(0).getLabel());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                    if (updatedRecognitions.size() > 0) {
                        if (updatedRecognitions.get(0).getLabel().equals("Quad")) { //if there is four rings
                            block.backward(0.3);
                            sleep(1800);
                            block.stop();

                            while (sensorRange2.getDistance(DistanceUnit.INCH) > 28) { //left
                                double power = 0.4;
                                telemetry.addData("angle", getAngle());
                                telemetry.update();
                                if (getAngle() < -1) { //turn left
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower((power + (0.01 * getAngle())));
                                    br.setPower((power + (0.01 * getAngle())));
                                } else if (getAngle() > 1) { //turn right
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower(power + (0.01 * getAngle()));
                                    br.setPower(power + (0.01 * getAngle()));
                                } else {
                                    tl.setPower(-power);
                                    tr.setPower(-power);
                                    bl.setPower(power);
                                    br.setPower(power);
                                }
                            }

                            block.stop();
                            arm.setPower(-0.5);
                            sleep(300);
                            arm.setPower(0);
                            shooter.setPower(-0.83);
                            transfer.setPower(0.5);
                            intake.setPower(1);
                            sleep(3000); //rev time
                            stopper.setPosition(0);
                            sleep(3000); //shoot time
                            shooter.setPower(0);
                            transfer.setPower(0);
                            intake.setPower(0);

                            while (sensorRange2.getDistance(DistanceUnit.INCH) > 8) { //left
                                double power = 0.4;
                                telemetry.addData("angle", getAngle());
                                telemetry.update();
                                if (getAngle() < -1) { //turn left
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower((power + (0.01 * getAngle())));
                                    br.setPower((power + (0.01 * getAngle())));
                                } else if (getAngle() > 1) { //turn right
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower(power + (0.01 * getAngle()));
                                    br.setPower(power + (0.01 * getAngle()));
                                } else {
                                    tl.setPower(-power);
                                    tr.setPower(-power);
                                    bl.setPower(power);
                                    br.setPower(power);
                                }
                            }
                            block.backward(0.5);
                            sleep(2000);
                            block.rightturn(0.5);
                            sleep(200);

                            block.stop();
                            arm.setPower(-0.5);
                            sleep(800);
                            arm.setPower(0);
                            sleep(500);
                            claw1.setPosition(0);
                            claw2.setPosition(1);
                            arm.setPower(0.5);
                            sleep(300);
                            arm.setPower(0);

                            block.forward(0.5);
                            sleep(1200);
                            block.stop();
                            stop();
                        }
                        if (updatedRecognitions.get(0).getLabel().equals("Single")) { //if there is one ring
                            block.backward(0.3);
                            sleep(1800);
                            block.stop();

                            while (sensorRange2.getDistance(DistanceUnit.INCH) > 28) { //left
                                double power = 0.4;
                                telemetry.addData("angle", getAngle());
                                telemetry.update();
                                if (getAngle() < -1) { //turn left
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower((power + (0.01 * getAngle())));
                                    br.setPower((power + (0.01 * getAngle())));
                                } else if (getAngle() > 1) { //turn right
                                    tl.setPower(-(power - (0.01 * getAngle())));
                                    tr.setPower(-(power - (0.01 * getAngle())));
                                    bl.setPower(power + (0.01 * getAngle()));
                                    br.setPower(power + (0.01 * getAngle()));
                                } else {
                                    tl.setPower(-power);
                                    tr.setPower(-power);
                                    bl.setPower(power);
                                    br.setPower(power);
                                }
                            }

                            block.stop();
                            arm.setPower(-0.5);
                            sleep(300);
                            arm.setPower(0);
                            shooter.setPower(-0.83);
                            transfer.setPower(0.5);
                            intake.setPower(1);
                            sleep(3000); //rev time
                            stopper.setPosition(0);
                            sleep(3000); //shoot time
                            shooter.setPower(0);
                            transfer.setPower(0);
                            intake.setPower(0);

                            block.backward(0.5);
                            sleep(1000);
                            block.stop();
                            arm.setPower(-0.5);
                            sleep(800);
                            arm.setPower(0);
                            sleep(500);
                            claw1.setPosition(0);
                            claw2.setPosition(1);
                            arm.setPower(0.5);
                            sleep(300);
                            arm.setPower(0);
                            block.forward(0.5);
                            sleep(500);
                            block.stop();
                            stop();
                        }
                    }
                    else { //if there are no rings
                        block.backward(0.3);
                        sleep(1800);
                        block.stop();

                        while (sensorRange2.getDistance(DistanceUnit.INCH)>28){ //left
                            double power = 0.4;
                            telemetry.addData("angle", getAngle());
                            telemetry.update();
                            if (getAngle() < -1){ //turn left
                                tl.setPower(-(power - (0.01 * getAngle())));
                                tr.setPower(-(power - (0.01 * getAngle())));
                                bl.setPower((power + (0.01 * getAngle())));
                                br.setPower((power + (0.01 * getAngle())));
                            } else if (getAngle() > 1){ //turn right
                                tl.setPower(-(power - (0.01 * getAngle())));
                                tr.setPower(-(power - (0.01 * getAngle())));
                                bl.setPower(power + (0.01 * getAngle()));
                                br.setPower(power + (0.01 * getAngle()));
                            } else {
                                tl.setPower(-power);
                                tr.setPower(-power);
                                bl.setPower(power);
                                br.setPower(power);
                            }
                        }

                        block.stop();
                        arm.setPower(-0.5);
                        sleep(300);
                        arm.setPower(0);
                        shooter.setPower(-0.83);
                        transfer.setPower(0.5);
                        intake.setPower(1);
                        sleep(3000); //rev time
                        stopper.setPosition(0);
                        sleep(3000); //shoot time
                        shooter.setPower(0);
                        transfer.setPower(0);
                        intake.setPower(0);

                        while (sensorRange2.getDistance(DistanceUnit.INCH)>12){ //left
                            double power = 0.4;
                            telemetry.addData("angle", getAngle());
                            telemetry.update();
                            if (getAngle() < -1){ //turn left
                                tl.setPower(-(power - (0.01 * getAngle())));
                                tr.setPower(-(power - (0.01 * getAngle())));
                                bl.setPower((power + (0.01 * getAngle())));
                                br.setPower((power + (0.01 * getAngle())));
                            } else if (getAngle() > 1){ //turn right
                                tl.setPower(-(power - (0.01 * getAngle())));
                                tr.setPower(-(power - (0.01 * getAngle())));
                                bl.setPower(power + (0.01 * getAngle()));
                                br.setPower(power + (0.01 * getAngle()));
                            } else {
                                tl.setPower(-power);
                                tr.setPower(-power);
                                bl.setPower(power);
                                br.setPower(power);
                            }
                        }
                        block.stop();
                        arm.setPower(-0.5);
                        sleep(800);
                        arm.setPower(0);
                        sleep(500);
                        claw1.setPosition(0);
                        claw2.setPosition(1);
                        arm.setPower(0.5);
                        sleep(300);
                        arm.setPower(0);
                        block.forward(0.5);
                        sleep(200);
                        while (sensorRange2.getDistance(DistanceUnit.INCH)<24){ //right
                            double power = 0.4;
                            telemetry.addData("angle", getAngle());
                            telemetry.update();
                            if (getAngle() < -1){ //turn left
                                tl.setPower((power + (0.01 * getAngle())));
                                tr.setPower((power + (0.01 * getAngle())));
                                bl.setPower(-(power - (0.01 * getAngle())));
                                br.setPower(-(power - (0.01 * getAngle())));
                            } else if (getAngle() > 1){ //turn right
                                tl.setPower((power + (0.01 * getAngle())));
                                tr.setPower((power + (0.01 * getAngle())));
                                bl.setPower(-(power - (0.01 * getAngle())));
                                br.setPower(-(power - (0.01 * getAngle())));
                            } else {
                                tl.setPower(power);
                                tr.setPower(power);
                                bl.setPower(-power);
                                br.setPower(-power);
                            }
                        }
                        block.backward(0.5);
                        sleep(600);
                        block.stop();
                        stop();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
