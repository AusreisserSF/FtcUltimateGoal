package org.firstinspires.ftc.teamcode.teleop.utility;

public class Button {

    public enum State // button states
    {
        DOWN,   // moment press down
        DOUBLE_TAP, // pressed down in quick succession
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    private static final int doubleTapIntervalMs = 500;
    private State state;
    private long lastTapped = -1;

    public Button() {
        state = State.NOT_INITIALIZED;
    }

    public State getState() {
        return state;
    }

    private boolean doubleTapIntervalNotSet() {
        return doubleTapIntervalMs == -1;
    }

    public State update(boolean buttonPressed) {
        if (buttonPressed) {
            if (state == State.OFF || state == State.UP || state == State.NOT_INITIALIZED) {
                if (System.currentTimeMillis() - lastTapped < doubleTapIntervalMs) {
                    state = State.DOUBLE_TAP;
                } else {
                    lastTapped = System.currentTimeMillis();
                    state = State.DOWN;
                }
            }
            else {
                state = State.HELD;
            }
        }
        else {
            if (state == State.HELD || state == State.DOWN || state == State.DOUBLE_TAP) {
                state = State.UP;
            }
            else {
                state = State.OFF;
            }
        }
        return state;
    }

    public boolean is(State state) {
        return this.state == state;
    }

}
