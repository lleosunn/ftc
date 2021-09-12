package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class teleop extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        motorblock block = new motorblock(tl, tr, bl, br);
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");

        boolean intakeMotorStatus = false;
        boolean intakeButtonStatus = false;
        boolean shooterMotorStatus = false;
        boolean shooterButtonStatus = false;

//        tl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        tr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        while (opModeIsActive()) {
            double power = 1;
            double turnpower = 0.7;
            if (gamepad1.right_bumper) { //slow mode
                power = 0.5;
                turnpower = 0.3;
            } else {
                power = 1;
                turnpower = 0.7;
            }
            if (gamepad1.left_stick_y < 0) { //goes forward
                block.forward(power);
                continue;
            }
            if (gamepad1.left_stick_y > 0) { //goes backward
                block.backward(power);
                continue;
            }
            if (gamepad1.left_stick_x < 0) { //goes left
                block.left(power);
                continue;
            }
            if (gamepad1.left_stick_x > 0) { //goes right
                block.right(power);
                continue;
            }
            if (gamepad1.right_stick_x < 0) { //turns left
                tl.setPower(-turnpower);
                tr.setPower(-turnpower);
                bl.setPower(-turnpower);
                br.setPower(-turnpower);
                continue;
            }
            if (gamepad1.right_stick_x > 0) { //turns right
                tl.setPower(turnpower);
                tr.setPower(turnpower);
                bl.setPower(turnpower);
                br.setPower(turnpower);
                continue;
            } else {
                block.stop();
            }
            if (gamepad1.x) {

            }
            if (gamepad1.left_trigger == 1 && !intakeButtonStatus) {
                intakeMotorStatus = !intakeMotorStatus;
                intakeButtonStatus = true;
            }
            if (gamepad1.left_trigger == 0) intakeButtonStatus = false;
            intake.setPower(intakeMotorStatus ? 1 : 0);
            transfer.setPower(intakeMotorStatus ? 0.6 : 0); //toggles intake and transfer

            if (gamepad1.y) { //reverses intake and transfer
                intake.setPower(-1);
                transfer.setPower(-0.6);
            }

            if (gamepad1.right_trigger == 1 && !shooterButtonStatus) {
                shooterMotorStatus = !shooterMotorStatus;
                shooterButtonStatus = true;
            }
            if (gamepad1.right_trigger == 0) shooterButtonStatus = false;
            shooter.setPower(shooterMotorStatus ? -0.85 : 0); //toggles shooter

            if (gamepad1.right_bumper) { //slow mode
                power = 0.5;
                turnpower = 0.3;
            } else {
                power = 1;
                turnpower = 0.7;
            }

            if (gamepad1.dpad_up) { //arm up
                arm.setPower(0.8);
            } else arm.setPower(0);

            if (gamepad1.dpad_down) { //arm down
                arm.setPower(-0.5);
            } else arm.setPower(0);

            if (gamepad1.dpad_left) { //claw open
                claw1.setPosition(1);
                claw2.setPosition(0);
            }
            if (gamepad1.dpad_right) { //claw close
                claw1.setPosition(0);
                claw2.setPosition(1);
            }
            if (gamepad1.a) { //shooting mode
                stopper.setPosition(0);
            }
            if (gamepad1.b) { //stopping mode
                stopper.setPosition(1);
            }

        }
    }
}

