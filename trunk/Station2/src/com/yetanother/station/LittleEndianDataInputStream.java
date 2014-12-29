package com.yetanother.station;

import java.io.IOException;

public class LittleEndianDataInputStream{

	public LittleEndianDataInputStream(byte[] in) {
		this.data = in;
		this.w = new byte[8];
	}


	public final short readShort() throws IOException
	{
		myReadFully(w, 0, 2);
		return (short)(
				(w[1]&0xff) << 8 |
				(w[0]&0xff));
	}

	/**
	 * Note, returns int even though it reads a short.
	 */
	 public final int readUnsignedShort() throws IOException
	 {
		 myReadFully(w, 0, 2);
		 return (
				 (w[1]&0xff) << 8 |
				 (w[0]&0xff));
	 }

	 /**
	  * like DataInputStream.readChar except little endian.
	  */
	 public final char readChar() throws IOException
	 {
		 myReadFully(w, 0, 2);
		 return (char) (
				 (w[1]&0xff) << 8 |
				 (w[0]&0xff));
	 }

	 /**
	  * like DataInputStream.readInt except little endian.
	  */
	 public final int readInt() throws IOException
	 {
		 myReadFully(w, 0, 4);
		 return
		 (w[3])      << 24 |
		 (w[2]&0xff) << 16 |
		 (w[1]&0xff) <<  8 |
		 (w[0]&0xff);
	 }

	 /**
	  * like DataInputStream.readLong except little endian.
	  */
	 public final long readLong() throws IOException
	 {
		 myReadFully(w, 0, 8);
		 return
		 (long)(w[7])      << 56 | 
		 (long)(w[6]&0xff) << 48 |
		 (long)(w[5]&0xff) << 40 |
		 (long)(w[4]&0xff) << 32 |
		 (long)(w[3]&0xff) << 24 |
		 (long)(w[2]&0xff) << 16 |
		 (long)(w[1]&0xff) <<  8 |
		 (long)(w[0]&0xff);
	 }

	 public final float readFloat() throws IOException {
		 return Float.intBitsToFloat(readInt());
	 }

	 public final double readDouble() throws IOException {
		 return Double.longBitsToDouble(readLong());
	 }

	 public final int read(byte b[], int off, int len) throws IOException {
		 return this.myReadFully(b, off, len);
	 }
	 
	 public final int skipBytes(int n) throws IOException {
		 pos += n;
		 return n;
	 }

	 public final boolean readBoolean() throws IOException {
		 return this.data[pos++] != 0;
	 }

	 public final byte readByte() throws IOException {
		 return this.data[pos++];
	 }

	 public final int readUnsignedByte() throws IOException {
		 return this.data[pos++] & 0xff;
	 }

	 
	 private int myReadFully(byte[] array, int start, int size){
		 for(int i=0; i<size; i++){
			 array[start+i] = this.data[pos++];
		 }
		 return size;
	 }
	 private byte w[]; // work array for buffering input
	 private byte data[];
	 private int pos = 0;
}


