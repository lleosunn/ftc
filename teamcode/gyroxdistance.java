package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Disabled
public class gyroxdistance extends LinearOpMode {

    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;

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
        motorblock block = new motorblock(tl, tr, bl, br);
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensor_range2");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorRange2;

        imuinit();
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("angle", getAngle());
            telemetry.update();

            if (sensorRange.getDistance(DistanceUnit.INCH) > 12) { //go forward
                double power = 0.2 + ((sensorRange.getDistance(DistanceUnit.INCH) - 12) * 0.06);
                if (getAngle() < -1) { //turn left
                    tl.setPower(power + (0.01 * getAngle()));
                    tr.setPower(-(power - (0.01 * getAngle())));
                    bl.setPower(power + (0.01 * getAngle()));
                    br.setPower(-(power - (0.01 * getAngle())));
                } else if (getAngle() > 1) { //turn right
                    tl.setPower(power + (0.01 * getAngle()));
                    tr.setPower(-(power - (0.01 * getAngle())));
                    bl.setPower(power + (0.01 * getAngle()));
                    br.setPower(-(power - (0.01 * getAngle())));
                } else {
                    tl.setPower(power);
                    tr.setPower(-power);
                    bl.setPower(power);
                    br.setPower(-power);
                }
            } else if (sensorRange.getDistance(DistanceUnit.INCH) < 11) { //go backward
                double power = 0.2 + ((11 - sensorRange.getDistance(DistanceUnit.INCH)) * 0.06);
                if (getAngle() < -1) { //turn left
                    tl.setPower(-(power - (0.01 * getAngle())));
                    tr.setPower((power + (0.01 * getAngle())));
                    bl.setPower(-(power - (0.01 * getAngle())));
                    br.setPower((power + (0.01 * getAngle())));
                } else if (getAngle() > 1) { //turn right
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
            } else {
                block.stop();
            }
            if (sensorRange2.getDistance(DistanceUnit.INCH) > 12) { //go left
                double power = 0.4 + ((sensorRange.getDistance(DistanceUnit.INCH) - 12) * 0.15);
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
            } else if (sensorRange2.getDistance(DistanceUnit.INCH) < 11) { //go right
                double power = 0.4 + ((11 - sensorRange2.getDistance(DistanceUnit.INCH)) * 0.15);
                if (getAngle() < -1) { //turn left
                    tl.setPower((power + (0.01 * getAngle())));
                    tr.setPower((power + (0.01 * getAngle())));
                    bl.setPower(-(power - (0.01 * getAngle())));
                    br.setPower(-(power - (0.01 * getAngle())));
                } else if (getAngle() > 1) { //turn right
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
            } else {
                block.stop();
            }


        }
    }
}