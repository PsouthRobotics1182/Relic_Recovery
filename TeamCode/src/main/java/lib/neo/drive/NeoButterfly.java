/** NeoButterfly
 * A NeoDrive implementation for a butterfly drivetrain.
 */

package lib.neo.drive;

import lib.neo.drive.core.NeoDrive;

public class NeoButterfly implements NeoDrive {
    @Override
    public boolean goes_fw(float mm) {
        return false;
    }

    @Override
    public void go_fw(float mm, Runnable r) {

    }

    @Override
    public void go_fw(float mm) {

    }

    @Override
    public boolean goes_le(float mm) {
        return false;
    }

    @Override
    public void go_le(float mm, Runnable r) {

    }

    @Override
    public void go_le(float mm) {

    }

    @Override
    public boolean goes_ri(float mm) {
        return false;
    }

    @Override
    public void go_ri(float mm, Runnable r) {

    }

    @Override
    public void go_ri(float mm) {

    }

    @Override
    public boolean rots(float rad) {
        return false;
    }

    @Override
    public void rot(float rad, Runnable r) {

    }

    @Override
    public void rot(float rad) {

    }
   /* NeoMotor wheel[][];
    NeoMotor wheelStrafe;

    public boolean goes_fw(float mm) {

    }

    public void go_fw(float mm) { go_fw(mm, new NeoRunnables.emptyRunnable()); }
    public void go_fw(float mm, Runnable r) {

    }

    public boolean goes_le(float mm) {

    }

    public void go_le(float mm) { go_le(mm, new NeoRunnables.emptyRunnable()); }
    public void go_le(float mm, Runnable r) {

    }

    public boolean goes_ri(float mm) {

    }

    public void go_ri(float mm) { go_ri(mm, new NeoRunnables.emptyRunnable()); }
    public void go_ri(float mm, Runnable r) {

    }

    public boolean rots(float rad) {

    }

    public void rot(float rad) { rot(rad, new NeoRunnables.emptyRunnable()); }
    public void rot(float rad, Runnable r) {

    }*/
}
