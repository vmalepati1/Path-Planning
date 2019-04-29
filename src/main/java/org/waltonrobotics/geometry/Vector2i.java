package org.waltonrobotics.geometry;

/**
 * Standard two-dimensional integer vector.
 */
public class Vector2i {

    private int x, y;

    /**
     * Creates a new zero vector.
     */
    public Vector2i() {
        set(0, 0);
    }

    /**
     * Creates new vector from another vector.
     *
     * @param vector: Other vector
     */
    public Vector2i(Vector2i vector) {
        set(vector.x, vector.y);
    }

    /**
     * .
     * Creates a new vector from x and y coordinates.
     *
     * @param x: X coordinate
     * @param y: Y coordinate
     */
    public Vector2i(int x, int y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Sets vector to specified x and y coordinates.
     *
     * @param x: X coordinate to set to
     * @param y: Y coordinate to set to
     */
    public void set(int x, int y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return X coordinate
     */
    public int getX() {
        return x;
    }

    /**
     * Sets the x coordinate.
     *
     * @param x: X coordinate to set to
     * @return The current vector
     */
    public Vector2i setX(int x) {
        this.x = x;
        return this;
    }

    /**
     * @return Y coordinate
     */
    public int getY() {
        return y;
    }

    /**
     * Sets the y coordinate.
     *
     * @param y: Y coordinate to set to
     * @return The current vector
     */
    public Vector2i setY(int y) {
        this.y = y;
        return this;
    }

    @Override
    public String toString() {
        return "Vector2i: (" + getX() + ", " + getY() + ")";
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Vector2i)) return false;
        Vector2i vec = (Vector2i) object;
        return vec.getX() == getX() && vec.getY() == getY();
    }

    @Override
    public int hashCode() {
        long result = 17;
        result = 31 * result + Double.doubleToLongBits(getX());
        result = 31 * result + Double.doubleToLongBits(getY());
        return (int) result;
    }

}
