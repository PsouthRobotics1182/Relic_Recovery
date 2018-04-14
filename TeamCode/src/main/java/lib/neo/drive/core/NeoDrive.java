package lib.neo.drive.core;

/**
 * Created by Robotics on 3/19/2018.
 */

public interface NeoDrive {
    boolean goes_fw(float mm);
    void go_fw(float mm, Runnable r);
    void go_fw(float mm);

    boolean goes_le(float mm);
    void go_le(float mm, Runnable r);
    void go_le(float mm);

    boolean goes_ri(float mm);
    void go_ri(float mm, Runnable r);
    void go_ri(float mm);

    boolean rots(float rad);
    void rot(float rad, Runnable r);
    void rot(float rad);
}
