package org.waltonrobotics.geometry;

import java.io.Serializable;

/**
 * Standard two-dimensional floating-point vector.
 */
public class Vector2f implements Serializable {

    private double x, y;
    private int polygonID;

    public int getPolygonID() {
        return polygonID;
    }

    public void setPolygonID(int polygonID) {
        this.polygonID = polygonID;
    }

    /**
     * Creates a new zero vector.
     */
    public Vector2f() {
        set(0, 0);
        this.polygonID = -1;
    }

    /**
     * Creates a new zero vector.
     */
    public Vector2f(int polygonID) {
        set(0, 0);
        this.polygonID = polygonID;
    }

    /**
     * Creates new vector from another vector.
     *
     * @param vector: Other vector
     */
    public Vector2f(Vector2f vector) {
        set(vector.x, vector.y);
        this.polygonID = vector.polygonID;
    }

    /**
     * Creates a new vector from x and y coordinates.
     *
     * @param x: X coordinate
     * @param y: Y coordinate
     */
    public Vector2f(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a new vector from x and y coordinates.
     *
     * @param x: X coordinate
     * @param y: Y coordinate
     */
    public Vector2f(double x, double y, int polygonID) {
        this.x = x;
        this.y = y;
        this.polygonID = polygonID;
    }

    /**
     * Creates a scalar vector.
     *
     * @param scalar: Scalar value
     */
    public Vector2f(double scalar) {
        this.x = scalar;
        this.y = scalar;
    }

    /**
     * Creates a scalar vector.
     *
     * @param scalar: Scalar value
     */
    public Vector2f(double scalar, int polygonID) {
        this.x = scalar;
        this.y = scalar;
        this.polygonID = polygonID;
    }

    /**
     * Sets vector to specified x and y coordinates.
     *
     * @param x: X coordinate to set to
     * @param y: Y coordinate to set to
     */
    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Sets this vector equivalent to another vector.
     *
     * @param other: Other vector
     */
    public void set(Vector2f other) {
        this.x = other.x;
        this.y = other.y;
    }

    /**
     * @return X coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Sets the x coordinate.
     *
     * @param x: X coordinate to set to
     * @return The current vector
     */
    public Vector2f setX(double x) {
        this.x = x;
        return this;
    }

    /**
     * @return Y coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the y coordinate.
     *
     * @param y: Y coordinate to set to
     * @return The current vector
     */
    public Vector2f setY(double y) {
        this.y = y;
        return this;
    }

    /**
     * Negates this vector.
     */
    public void negate() {
        x *= -1;
        y *= -1;
    }

    /**
     * Adds this vector and another vector and returns the sum vector.
     *
     * @param other: Another vector to add with
     * @return Sum vector
     */
    public Vector2f add(Vector2f other) {
        return new Vector2f(x + other.x, y + other.y);
    }

    /**
     * Adds a value to x and y coordinates and returns this new sum vector.
     *
     * @param value: Value to add
     * @return Sum vector
     */
    public Vector2f add(double value) {
        return add(new Vector2f(value));
    }

    /**
     * Subtracts another vector from this vector and returns the difference vector.
     *
     * @param other: Another vector to subtract with
     * @return Difference vector
     */
    public Vector2f subtract(Vector2f other) {
        return new Vector2f(x - other.x, y - other.y);
    }

    /**
     * Subtracts a value from x and y coordinates and returns this new difference vector.
     *
     * @param value: Value to subtract
     * @return Difference vector
     */
    public Vector2f subtract(double value) {
        return subtract(new Vector2f(value));
    }

    /**
     * Multiplies another vector with this vector and returns the product vector.
     *
     * @param other: Another vector to multiply with
     * @return Product vector
     */
    public Vector2f multiply(Vector2f other) {
        return new Vector2f(x * other.x, y * other.y);
    }

    /**
     * Multiplies a value with x and y coordinates and returns this new product vector.
     *
     * @param value: Value to multiply
     * @return Product vector
     */
    public Vector2f multiply(double value) {
        return multiply(new Vector2f(value));
    }

    /**
     * Divides this vector with another vector and returns the quotient vector.
     *
     * @param other: Another vector to divide with
     * @return Quotient vector
     */
    public Vector2f divide(Vector2f other) {
        return new Vector2f(x / other.x, y / other.y);
    }

    /**
     * Divides this vector's x and y coordinates with a value and returns this new quotient vector.
     *
     * @param value: Value to divide with
     * @return Quotient vector
     */
    public Vector2f divide(double value) {
        return divide(new Vector2f(value));
    }

    /**
     * Performs dot product with this vector and another vector.
     *
     * @param other: Another vector to perform dot product with
     * @return Dot product of two vectors
     */
    public double dot(Vector2f other) {
        return x * other.x + y * other.y;
    }


    /**
     * Finds distance between this point and another point
     * @param other: Other point
     * @return Distance from this point to other point
     */
    public double distanceTo(Vector2f other) {
        return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
    }

    /**
     * Performs vector triple product using expansion.
     *
     * @param vector2: Second vector
     * @param vector3: Third vector
     * @return Triple product of vectors
     */
    public Vector2f tripleProduct(Vector2f vector2, Vector2f vector3) {
        Vector2f firstHalf = vector2.multiply(dot(vector3));
        Vector2f secondHalf = vector3.multiply(dot(vector2));

        return firstHalf.subtract(secondHalf);
    }

    /**
     * @return Magnitude of this vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * @return Normalized vector of this vector
     */
    public Vector2f normalize() {
        double length = magnitude();
        return new Vector2f(x / length, y / length);
    }

    /**
     * @return Negative of this vector
     */
    public Vector2f negative() {
        return new Vector2f(-x, -y);
    }

    @Override
    public String toString() {
        return "Vector2f: (" + x + ", " + y + ")";
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Vector2f)) return false;
        Vector2f vec = (Vector2f) object;
        return vec.getX() == this.getX() && vec.getY() == this.getY();
    }

    @Override
    public int hashCode() {
        long result = 17;
        result = 31 * result + Double.doubleToLongBits(x);
        result = 31 * result + Double.doubleToLongBits(y);
        return (int) result;
    }

    /**
     * Simple comparison function for comparing doubles.
     *
     * @param v1: Value 1
     * @param v2: Value 2
     * @return -1 if v1 < v2, 0 if v1 == v2, 1 if v2 > v2
     */
    private int compareValues(double v1, double v2) {
        if (v1 > v2) {
            return 1;
        } else if (v1 < v2) {
            return -1;
        }

        return 0;
    }

    /**
     * Compares x values of this vector and another vector.
     *
     * @param other: Another vector to compare x values with
     * @return -1 if this.x < other.x, 0 if this.x == other.x, 1 if this.x > other.x
     */
    public int compareX(Vector2f other) {
        return compareValues(this.x, other.getX());
    }

    /**
     * Compares y values of this vector and another vector.
     *
     * @param other: Another vector to compare y values with
     * @return -1 if this.y < other.y, 0 if this.y == other.y, 1 if this.y > other.y
     */
    public int compareY(Vector2f other) {
        return compareValues(this.y, other.getY());
    }

}
