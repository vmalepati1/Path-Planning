package org.waltonrobotics.geometry;

public class Rectangle {

    private double minX;
    private double maxX;
    private double minY;
    private double maxY;

    private Vector2f centerPoint;

    public Rectangle(double minX, double maxX, double minY, double maxY) {
        setBounds(minX, maxX, minY, maxY);
    }

    public Rectangle(Rectangle rectangle) {
        setBounds(rectangle.minX, rectangle.maxX, rectangle.minY, rectangle.maxY);
    }

    public Rectangle() {
        setToNull();
    }

    public double getMinX() {
        return minX;
    }

    public double getMaxX() {
        return maxX;
    }

    public double getMinY() {
        return minY;
    }

    public double getMaxY() {
        return maxY;
    }

    public Vector2f getCenterPoint() {
        return centerPoint;
    }

    public void setBounds(double minX, double maxX, double minY, double maxY) {
        if (minX < maxX && minY < maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
        else {
            setToNull();
        }

        calculateCenterPoint();
    }

    public void setToNull() {
        this.minX = 0;
        this.maxX = -1;
        this.minY = 0;
        this.maxY = -1;
    }

    public boolean isNull() {
        return maxX < minX;
    }

    public void calculateCenterPoint() {
        if (isNull()) return;
        centerPoint = new Vector2f((minX + maxX) / 2, (minY + maxY) / 2);
    }

    public void expandToInclude(Rectangle other) {
        if (other.isNull()) return;

        if (isNull()) {
            setBounds(other.minX, other.maxX, other.minY, other.maxY);
        } else {
            if (other.minX < minX) {
                minX = other.minX;
            }

            if (other.maxX > maxX) {
                maxX = other.maxX;
            }

            if (other.minY < minY) {
                minY = other.minY;
            }

            if (other.maxY > maxY) {
                maxY = other.maxY;
            }
        }
    }

    public boolean intersects(Rectangle other) {
        if (isNull() || other.isNull()) return false;
        return other.minX <= maxX && other.maxX >= minX && other.minY <= other.maxY && other.maxY >= other.minY;
    }

    @Override
    public String toString() {
        return "Rectangle: [lower-left: " + new Vector2f(getMinX(), getMinY()) + ", top-right: " + new Vector2f(getMaxX(), getMaxY()) + "]";
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Rectangle)) return false;
        Rectangle rect = (Rectangle) object;
        return rect.getMinX() == getMinX() && rect.getMaxX() == getMaxX()
                && rect.getMinY() == getMinY() && rect.getMaxY() == getMaxY();
    }

    @Override
    public int hashCode() {
        long result = 17;
        result = 31 * result + Double.doubleToLongBits(getMinX());
        result = 31 * result + Double.doubleToLongBits(getMaxX());
        result = 31 * result + Double.doubleToLongBits(getMinY());
        result = 31 * result + Double.doubleToLongBits(getMaxY());
        return (int) result;
    }

}
