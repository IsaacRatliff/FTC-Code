package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Stone Delivery 2", group="Linear Opmode")

public class Stones2 extends AutoOp {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(1.07);
        closeClaw();
        pause(1.8);
        armUp();
        backward(0.8);
        right(1);
        armDown();
        forward(3.5);
        openClaw();
        left(2.2);
        forward(1.23);
        right(1);
        forward(2.45);
        right(1);
        forward(1);
        right(1.2);
        forward(1.9);
        
        stopWheels();
        
        
    }
}
