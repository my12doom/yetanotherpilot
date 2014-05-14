package com.yetanother.station;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.UUID;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuItem.OnMenuItemClickListener;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity {

	private static final UUID COM_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
	private BluetoothAdapter mBluetoothAdapter = null;
	private BluetoothSocket btSocket = null;
	private OutputStream outStream = null;
	private InputStream inStream = null;
	private static String address = "00:14:01:06:17:16";
	private static String TAG = "yetanotherstation";
	private StringBuilder sb = new StringBuilder();
	private byte[] recvbuf = new byte[64];
	private byte[] payload = new byte[32];
	private byte[][] packets = new byte[256][32];
	private Map<String, Double> latest_value = new LinkedHashMap<String, Double>();
	private boolean mBeforeBTState = false;
	WorkerThread mWorker;
	int recvsize = 0;
	float pilot_time = 0;
	private TextView tv;
	
	private float p=-10000,i=-10000,d=-10000;
	private float p2=-10000,i2=-10000,d2=-10000;
	private float p3=-10000,i3=-10000,d3=-10000;
	private float p4=-10000,i4=-10000,d4=-10000;
	private float roll=-10000, pitch=-10000, yaw=-10000;
	private int level = 0xffffffff;
	
		
	Long TAG_SENSOR_DATA = (long) 0x12;
	Long TAG_IMU_DATA = (long) 0x87;
	Long TAG_PILOT_DATA = (long) 0x65;
	Long TAG_PILOT_DATA2 = (long) 0x66;
	Long TAG_PPM_DATA = (long) 0x33;
	Long TAG_CTRL_DATA = (long) 0x34;
	Long TAG_GPS_DATA_V1 = (long) 0x35;
	Long TAG_GPS_DATA = (long) 0x36;
	
	private Handler handler = new Handler(){
		public void handleMessage(Message msg){
			
			parse_packets();
			
			synchronized(latest_value){
				try{
				StringBuilder sb = new StringBuilder();
					sb.append(""+pilot_time+"\n");
					Iterator iter = latest_value.entrySet().iterator();
					while (iter.hasNext()) {
						Map.Entry entry = (Map.Entry) iter.next();
						sb.append( entry.getKey().toString() + " : " + entry.getValue() + "\n");
					}
					tv.setText(sb.toString());
				}catch(Throwable e){}
			}
				
			// read textboxes
			try{
				level = (int) Long.parseLong(((TextView)findViewById(R.id.et_log)).getText().toString());
			}catch(Exception e){};
			try{
				p = Float.parseFloat(((TextView)findViewById(R.id.et_p)).getText().toString());
			}catch(Exception e){};
			try{
				i = Float.parseFloat(((TextView)findViewById(R.id.et_i)).getText().toString());
			}catch(Exception e){};
			try{
				d = Float.parseFloat(((TextView)findViewById(R.id.et_d)).getText().toString());
			}catch(Exception e){};
			try{ 
				p2 = Float.parseFloat(((TextView)findViewById(R.id.et_p2)).getText().toString());
			}catch(Exception e){};
			try{
				i2 = Float.parseFloat(((TextView)findViewById(R.id.et_i2)).getText().toString());
			}catch(Exception e){};
			try{
				d2 = Float.parseFloat(((TextView)findViewById(R.id.et_d2)).getText().toString());
			}catch(Exception e){};

			try{
				p3 = Float.parseFloat(((TextView)findViewById(R.id.et_p3)).getText().toString());
			}catch(Exception e){};
			try{
				i3 = Float.parseFloat(((TextView)findViewById(R.id.et_i3)).getText().toString());
			}catch(Exception e){};
			try{
				d3 = Float.parseFloat(((TextView)findViewById(R.id.et_d3)).getText().toString());
			}catch(Exception e){};

			try{
				p4 = Float.parseFloat(((TextView)findViewById(R.id.et_p4)).getText().toString());
			}catch(Exception e){};
			try{
				i4 = Float.parseFloat(((TextView)findViewById(R.id.et_i4)).getText().toString());
			}catch(Exception e){};
			try{
				d4 = Float.parseFloat(((TextView)findViewById(R.id.et_d4)).getText().toString());
			}catch(Exception e){};

			try{
				roll = Float.parseFloat(((TextView)findViewById(R.id.et_roll)).getText().toString());
			}catch(Exception e){};
			try{
				pitch = Float.parseFloat(((TextView)findViewById(R.id.et_pitch)).getText().toString());
			}catch(Exception e){};
			try{
				yaw = Float.parseFloat(((TextView)findViewById(R.id.et_yaw)).getText().toString());
			}catch(Exception e){};
  		}
	};
	
	private String[][] fieldss = new String[][]
	{
		{TAG_SENSOR_DATA.toString(), 
			"short mag0", "short mag1", "short mag2",
			"short accel0", "short accel1", "short accel2",
			"short temperature1",
			"short gyro0", "short gyro1", "short gyro2",
			"short voltage",
			"short current",
		},
		{TAG_IMU_DATA.toString(),
			"unsigned short temperature",
			"unsigned short .ph",
			"int pressure",
			"short estAccGyro0", "short estAccGyro1", "short estAccGyro2",
			"short estGyro0", "short estGyro1", "short estGyro2",
			"short estMagGyro0", "short estMagGyro1", "short estMagGyro2",
		},
		{TAG_PILOT_DATA.toString(),
			"int altitude",							// 4 byte, unit base: 0.01 meter relative to launch ground.
			"float airspeed",
			"short error0",	"short error1", "short error2",						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
			"short target0", "short target1", "short target2", 						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
			"unsigned char fly_mode",					// 1 byte
		},
		{TAG_PILOT_DATA2.toString(),
			"int I0", "int I1", "int I2", 
			"int D0", "int D1", "int D2", 
		},
		{TAG_PPM_DATA.toString(),
			"short ppmi[1]", "short ppmi[2]", "short ppmi[3]", "short ppmi[4]", "short ppmi[5]", "short ppmi[6]",
			"short ppmo[1]", "short ppmo[2]", "short ppmo[3]", "short ppmo[4]", "short ppmo[5]", "short ppmo[6]", 
		},
		{TAG_GPS_DATA.toString(),
			"unsigned short PDOP", "unsigned short HDOP", "unsigned short VDOP",				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
			"short speed",					// unit: cm/s
			"float longitude",				// longitude in NDEG - +/-[degree][min].[sec/60]
			"float latitude",					// latitude in NDEG - +/-[degree][min].[sec/60]
			"float altitude",					// meter
			"unsigned short .satelite_info",
			//unsigned satelite_in_view 	: 4;
			//unsigned satelite_in_use	: 4;
			//unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
			//unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
		},
	};
	
	private enum datatype {
		_short,
		_ushort,
		_char,
		_uchar,
		_int,
		_uint,
		_float,		
	};
	
	private class parser {
		int tag = 0;
		datatype[] types;
		String[] names;
		
		public int parse(Map<String, Double> table, int tag, LittleEndianDataInputStream reader){
			
			if (tag != this.tag)
				return -1;
			
			for(int i=0; i<names.length; i++){
				double v = 0;
				try{
					switch(types[i]){
					case _short:
						v = reader.readShort();
						break;
					case _ushort:
						v = reader.readUnsignedShort();
						break;
					case _char:
						v = reader.readChar();
						break;
					case _uchar:
						v = reader.readUnsignedByte();
						break;
					case _int:
						v = reader.readInt();
						break;
					case _uint:
						v = reader.readInt();
						break;
					case _float:
						v = reader.readFloat();
						break;
					default:
						return -1;
					}
				}catch(Exception e){}
				
				table.put(names[i], v);
			}
			
			return 0;
		}
		public parser(){
			
		}
		
		public parser(String[] fields){
			if (fields.length<2)
				return;
			
			tag = Integer.parseInt(fields[0]);
			types = new datatype[fields.length-1];
			names = new String[fields.length-1];
			
			for(int i=1; i<fields.length; i++){
				String []desc = fields[i].split(" ");
				if (desc.length == 3){
					desc[0] += " " + desc[1];
					desc[1] = desc[2];
				}
				
				names[i-1] = desc[1];
				
				if ("short".equals(desc[0])){
					types[i-1] = datatype._short;
				}
				else if ("unsigned short".equals(desc[0])){
					types[i-1] = datatype._ushort;
				}
				else if ("unsigned char".equals(desc[0])){
					types[i-1] = datatype._uchar;
				}
				else if ("unsigned int".equals(desc[0])){
					types[i-1] = datatype._uint;
				}
				else if ("int".equals(desc[0])){
					types[i-1] = datatype._int;
				}
				else if ("float".equals(desc[0])){
					types[i-1] = datatype._float;
				}
				else {
					Log.e(TAG, "unknown data type " + desc[0]);
				}
			}
		}
	};
	

	private parser[] parsers;	
	
	@Override
	public void onPause(){
		super.onPause();
		try {
			getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
			
			if (!mBeforeBTState)
				mBluetoothAdapter.disable();
			outStream.close();
			inStream.close();
			btSocket.close();
			mWorker.exit();
			mWorker = null;
		} catch (IOException e) {
		}
	}
	
	long tick = System.currentTimeMillis();
	long tick2 = System.currentTimeMillis();
	int packet_speed = 0;
	
	private void store_packets(){
		synchronized(latest_value){
			if (recvsize != 32)		// invalid packets
				return;
			int tag = recvbuf[0] & 0xff;
			for(int i=0; i<32; i++)
				packets[tag][i] = recvbuf[i];
			packet_speed ++;
			if (System.currentTimeMillis() > tick + 1000){
				tick = System.currentTimeMillis();
				latest_value.put("packet_speed", (double)packet_speed);
				packet_speed = 0;
			}
		}
	}
	
	private void parse_packets(){
		for(int k=0; k<256; k++){
			if (k != packets[k][0])
				continue;
			LittleEndianDataInputStream reader = new LittleEndianDataInputStream(packets[k]);
			
			synchronized(latest_value){
	
				try {
					long time = reader.readLong();
					int tag = (int) (time >> 56) & 0xff;
					time = (time << 8 ) >> 8;
					
					for(int i=0; i<parsers.length; i++){
						parsers[i].parse(latest_value, tag, reader);						
					}
					
					/*
					
					String tag_str = String.format("0x%02x", tag);
					
					if (latest_value.get(tag_str) != null)				
						latest_value.put(tag_str, latest_value.get(tag_str)+1);
					else
						latest_value.put(tag_str, 0.0);
					
					pilot_time = (float) (time/1000000.0);
					*/
										
				} catch (Exception e) {
				}
			}
		}
	}
	
	
	private class WorkerThread extends Thread{
		private boolean exit = false;
		private long last_send_time = System.currentTimeMillis();
		public void run(){
			while (!exit){
				try {
					while(inStream.available() > 0){
						int got = inStream.read();
						if (got == '\r'){
							got = inStream.read();
							if (got == '\r')
								recvbuf[recvsize++] = '\r';
							if (got == '\n'){
								store_packets();
								recvsize = 0;
							}
						}
						else{
							recvbuf[recvsize++] = (byte) got;
						}
						if (System.currentTimeMillis() > tick2 + 50){
							tick2 = System.currentTimeMillis();
							handler.sendEmptyMessage(0);
						}
					}
					
					if (System.currentTimeMillis() > tick2 + 50){
						tick2 = System.currentTimeMillis();
						handler.sendEmptyMessage(0);
					}
					
					// do command sending
					if (System.currentTimeMillis() - last_send_time > 250){
						last_send_time = System.currentTimeMillis();
						
						outStream.write(String.format("log %d\r\n", level).getBytes());
						if (p>=-9999 && i>=-9999 && d>=-9999)
							outStream.write(String.format("pid %f %f %f\r\n", p, i, d).getBytes());
						if (p2>=-9999 && i2>=-9999 && d2>=-9999)
							outStream.write(String.format("pid2 %f %f %f\r\n", p2, i2, d2).getBytes());
						if (p3>=-9999 && i3>=-9999 && d3>=-9999)
							outStream.write(String.format("pid3 %f %f %f\r\n", p3, i3, d3).getBytes());
						if (p4>=-9999 && i4>=-9999 && d4>=-9999)
							outStream.write(String.format("pid4 %f %f %f\r\n", p4, i4, d4).getBytes());
						if (roll>=-9999 &&pitch >= -9999 && yaw >= -9999)
							outStream.write(String.format("trim %f %f %f\r\n", roll*Math.PI/180, pitch*Math.PI/180, yaw*Math.PI/180).getBytes());
					}
					Thread.sleep(10);
					
				}catch (Throwable e) {
				}
			} 
		}
		
		public void exit(){
			this.exit = true;
			try {
				this.join();
			} catch (InterruptedException e) {
			}
		}
	};
	
	@Override
	public void onResume(){
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
			Toast.makeText(this, "Bluetooth is not available.",	Toast.LENGTH_LONG).show();
			finish();
			return;
		}
		mBeforeBTState = mBluetoothAdapter.isEnabled();
		if (!mBeforeBTState) {
			mBluetoothAdapter.enable();
		}
		super.onResume();
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		connect();
	}
	
	private void connect(){

		BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
		if (mBluetoothAdapter == null) {
			Toast.makeText(this, "Can't get remote device.", Toast.LENGTH_LONG).show();
			finish();
			return;
		}

		
		try {
			btSocket = device.createRfcommSocketToServiceRecord(COM_UUID);
		} catch (IOException e) {
			Log.e(TAG, "ON RESUME: Socket creation failed.", e);
		}

		try {
			btSocket.connect();
			Log.e(TAG, "ON RESUME: BT connection established, data transfer link open.");
		} catch (IOException e) {
			try {
				btSocket.close();
			} catch (IOException e2) {
				Log.e(TAG, "ON RESUME: Unable to close socket during connection failure", e2);
			}
		}

		try {   
			outStream = btSocket.getOutputStream();			  
			inStream = btSocket.getInputStream(); //可在TextView里显示 			  
		} catch (IOException e) {
		    Log.e(TAG, "ON RESUME: Output stream creation failed.", e);
		}
		
		mWorker = new WorkerThread();
		mWorker.start();
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		tv = (TextView) this.findViewById(R.id.tv_hello);
		
		parsers = new parser[fieldss.length];
		for(int i=0; i<parsers.length; i++)
			parsers[i] = new parser(fieldss[i]);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		
		menu.findItem(R.id.connect).setOnMenuItemClickListener(new OnMenuItemClickListener(){
			public boolean onMenuItemClick(MenuItem item) {
				connect();
				return true;
			}
		});
		return true;
	}

}
