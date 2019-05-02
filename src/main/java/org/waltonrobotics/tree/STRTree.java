package org.waltonrobotics.tree;

import org.waltonrobotics.geometry.Rectangle;

import java.util.*;

public class STRTree {

    private int leafCapacity;
    private List<Rectangle> boundingBoxes;
    private STRTreeNode root;

    private static Comparator<Rectangle> xComparator = (r1, r2) -> r1.getCenterPoint().compareX(r2.getCenterPoint());
    private static Comparator<Rectangle> yComparator = (r1, r2) -> r1.getCenterPoint().compareY(r2.getCenterPoint());

    public STRTree(int leafCapacity, List<Rectangle> boundingBoxes) {
        this.leafCapacity = leafCapacity;
        this.boundingBoxes = boundingBoxes;
        calculateTree();
    }

    public int getLeafCapacity() {
        return leafCapacity;
    }

    public List<Rectangle> getBoundingBoxes() {
        return boundingBoxes;
    }

    public STRTreeNode getRoot() {
        return root;
    }

    private void addRoot(Rectangle boundingBox) {
        if (root == null) {
            root = new STRTreeNode(null, new LinkedList<>(), this, boundingBox);
        } else {
            throw new IllegalStateException("Trying to add a root node to a non-empty tree.");
        }
    }

    private STRTreeNode addNode(STRTreeNode parent, Rectangle boundingBox) {
        if (parent == null) {
            throw new NullPointerException("Cannot add child to null parent.");
        } else if (parent.hostTree != this) {
            throw new IllegalArgumentException("Parent node not a part of this tree.");
        } else {
            STRTreeNode newNode = new STRTreeNode(parent, new LinkedList<>(), this, boundingBox);
            parent.children.addLast(newNode);
            return newNode;
        }
    }

    private void calculateTree() {
        int leafLevelPages = (int)Math.ceil(boundingBoxes.size() / (double)leafCapacity);
        int numberOfVerticleSlices = (int)Math.ceil(Math.sqrt(leafLevelPages));
        int sliceCapacity = numberOfVerticleSlices * leafCapacity;

        Rectangle totalEncompassing = new Rectangle();

        for (Rectangle r : boundingBoxes) {
            totalEncompassing.expandToInclude(r);
        }

        addRoot(totalEncompassing);

        boundingBoxes.sort(xComparator);

        Iterator i = boundingBoxes.iterator();

        for (int j = 0; j < numberOfVerticleSlices; j++) {
            int boundingBoxesAddedToSlice = 0;

            Rectangle sliceEncompassing = new Rectangle();

            List<Rectangle> sliceBoundingBoxes = new ArrayList<>();

            while (i.hasNext() && boundingBoxesAddedToSlice < sliceCapacity) {
                Rectangle childBoundingBox = (Rectangle) i.next();
                sliceEncompassing.expandToInclude(childBoundingBox);
                sliceBoundingBoxes.add(childBoundingBox);
                boundingBoxesAddedToSlice++;
            }

            STRTreeNode sliceNode = addNode(root, sliceEncompassing);

            sliceBoundingBoxes.sort(yComparator);

            Iterator k = sliceBoundingBoxes.iterator();

            int boundingBoxesAddedToTile = 0;
            while (k.hasNext() && boundingBoxesAddedToTile < leafCapacity) {
                Rectangle childBoundingBox = (Rectangle) k.next();
                addNode(sliceNode, childBoundingBox);
                boundingBoxesAddedToTile++;
            }
        }

    }

    public static void printTree(STRTreeNode node) {
        System.out.println(node.boundingBox);

        for (STRTreeNode child : node.children) {
            printTree(child);
        }
    }

    public static void main(String[] args) {
        List<Rectangle> boundingBoxes = new ArrayList<>();

        boundingBoxes.add(new Rectangle());
        boundingBoxes.add(new Rectangle(-2, 2, -2, 2));
        boundingBoxes.add(new Rectangle(4, 5, 4, 5));
        boundingBoxes.add(new Rectangle(6, 7, 6, 7));
        boundingBoxes.add(new Rectangle(8, 9, 8, 9));
        boundingBoxes.add(new Rectangle(10, 11, 10, 11));
        boundingBoxes.add(new Rectangle(12, 13, 12, 13));

        STRTree strTree = new STRTree(2, boundingBoxes);

        printTree(strTree.getRoot());
    }

}
