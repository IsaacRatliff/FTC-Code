package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Reposition2 (Red)", group="Linear Opmode")

public class Reposition2Red extends AutoOp {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();  
        forward(1);
        right(1.8);
        //backward(0.125);
        fingerDown();
        forward(1.5);  // 1 tile at half spd
        fingerUp();
        forward(0.5);
        right(1);
        forward(1);
        right(1.5);
        /*forward(2);
        right(1);
        forward(3);
        backward(0.5);
        right(1);
        forward(2.5);*/
        // el fin
    }
}
