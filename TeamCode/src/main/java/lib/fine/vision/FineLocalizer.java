package lib.fine.vision;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * Created by drew on 1/31/18.
 */

public class FineLocalizer extends VuforiaLocalizerImpl {
    boolean closed = false;
    public FineLocalizer(Parameters parameters) {
        super(parameters);
    }
    @Override
    public void close() {
        if (!closed) super.close();
        closed = true;
    }

}