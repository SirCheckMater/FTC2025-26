package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;
import java.util.List;

/**
 * A simple ring buffer implementation for storing historical values.
 */
public class RingBuffer<T> {
    private List<T> list;
    private int index = 0;
    private int size;

    /**
     * Initializes the ring buffer with a specified length and starting value.
     *
     * @param length        The maximum number of elements in the buffer.
     * @param startingValue The initial value to populate the buffer.
     */
    public RingBuffer(int length, T startingValue){
        list = new ArrayList<>(length);
        for (int i = 0; i < length; i++) {
            list.add(startingValue);
        }
        size = length;
    }

    /**
     * Retrieves the oldest value in the buffer and inserts the current value.
     *
     * @param current The current value to insert into the buffer.
     * @return The oldest value that was replaced.
     */
    public T getValue(T current) {
        T retVal = list.get(index);
        list.set(index, current);
        index = (index + 1) % size;
        return retVal;
    }

    /**
     * Fills the entire buffer with a specific value.
     *
     * @param overwriteVal The value to overwrite all elements in the buffer.
     */
    public void fill(T overwriteVal){
        for (int i = 0; i < size; i++) {
            list.set(i, overwriteVal);
        }
    }

    /**
     * Retrieves the entire list of elements in the buffer.
     *
     * @return The list containing all buffer elements.
     */
    public List<T> getList(){
        return list;
    }

    /**
     * Changes the length of the buffer and fills it with a specific value.
     *
     * @param length       The new length of the buffer.
     * @param overwriteVal The value to fill the new buffer.
     */
    public void changeLength(int length, T overwriteVal){
        list = new ArrayList<>(length);
        for (int i = 0; i < length; i++) {
            list.add(overwriteVal);
        }
        size = length;
        index = 0;
    }
}
