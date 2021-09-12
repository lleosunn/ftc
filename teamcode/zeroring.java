package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


@Autonomous
public class zeroring extends LinearOpMode {

    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

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

    public void runOpMode() throws InterruptedException {
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
        waitForStart();
        while (opModeIsActive()) {
            stopper.setPosition(1);
            claw1.setPosition(0);
            claw2.setPosition(0);
            while (sensorRange3.getDistance(DistanceUnit.INCH)<60) { //back
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
            block.backward(0.3);
            sleep(1200);
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
            sleep(500);
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
            sleep(500);
            block.stop();


            stop();


        }
    }
}