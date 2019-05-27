package org.waltonrobotics.geometry.clipping;

import org.waltonrobotics.geometry.Vector2f;

public class Segment {

    private Vector2f start;
    private Vector2f end;

    public Segment(Vector2f start, Vector2f end) {
        setStart(start);
        setEnd(end);
    }

    public Vector2f getStart() {
        return start;
    }

    public void setStart(Vector2f start) {
        this.start = start;
    }

    public Vector2f getEnd() {
        return end;
    }

    public void setEnd(Vector2f end) {
        this.end = end;
    }

}
