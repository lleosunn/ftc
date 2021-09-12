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

public class motorblock {
    DcMotor tl;
    DcMotor tr;
    DcMotor bl;
    DcMotor br;

    public motorblock(DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br) {
        this.tl = tl;
        this.tr = tr;
        this.bl = bl;
        this.br = br;
    }

    public void leftturn(double power) {
        tl.setPower(-power);
        tr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    public void rightturn(double power) {
        tl.setPower(power);
        tr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void forward(double power) {
        tl.setPower(power);
        tr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }

    public void backward(double power) {
        tl.setPower(-power);
        tr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }

    public void left(double power) {
        tl.setPower(-power);
        tr.setPower(-power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void right(double power) {
        tl.setPower(power);
        tr.setPower(power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    public void stop() {
        tl.setPower(0);
        tr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
