package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class leosopmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor tl=hardwareMap.get(DcMotor.class,"tl");
        DcMotor tr=hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl=hardwareMap.get(DcMotor.class, "bl");
        DcMotor br=hardwareMap.get(DcMotor.class, "br");
        waitForStart();
        while (opModeIsActive()) {
            tl.setPower(-0.5);
            tr.setPower(-0.5);
            bl.setPower(-0.5);
            br.setPower(-0.5);


        }
    }
}
