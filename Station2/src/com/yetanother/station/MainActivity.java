package com.yetanother.station;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;
import android.os.Bundle;
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

public class MainActivity extends Activity
{

	private static final UUID COM_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
	private BluetoothAdapter mBluetoothAdapter = null;
	private BluetoothSocket btSocket = null;
	private OutputStream outStream = null;
	private InputStream inStream = null;
	private static String address = "00:14:01:06:17:16";
	private static String TAG = "yetanotherstation";
	private boolean mBeforeBTState = false;
	int recvsize = 0;
	float pilot_time = 0;
	private TextView tv;
	private CLI cli = new CLI();

	private void disconnect()
	{
		try
		{
			outStream.close();
			inStream.close();
			btSocket.close();
		} catch (Exception e)
		{
		}
	}

	@Override
	public void onPause()
	{
		super.onPause();
		try
		{
			getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

			if (!mBeforeBTState)
				mBluetoothAdapter.disable();
			outStream.close();
			inStream.close();
			btSocket.close();
		} catch (IOException e)
		{
		}
	}

	@Override
	public void onResume()
	{
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null)
		{
			Toast.makeText(this, "Bluetooth is not available.", Toast.LENGTH_LONG).show();
			finish();
			return;
		}
		mBeforeBTState = mBluetoothAdapter.isEnabled();
		if (!mBeforeBTState)
		{
			mBluetoothAdapter.enable();
		}
		super.onResume();
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		connect();
	}

	private void connect()
	{
		disconnect();

		BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
		if (mBluetoothAdapter == null)
		{
			Toast.makeText(this, "Can't get remote device.", Toast.LENGTH_LONG).show();
			finish();
			return;
		}

		try
		{
			btSocket = device.createRfcommSocketToServiceRecord(COM_UUID);
		} catch (IOException e)
		{
			Log.e(TAG, "ON RESUME: Socket creation failed.", e);
		}

		try
		{
			btSocket.connect();
			Log.e(TAG, "ON RESUME: BT connection established, data transfer link open.");
		} catch (IOException e)
		{
			try
			{
				btSocket.close();
			} catch (IOException e2)
			{
				Log.e(TAG, "ON RESUME: Unable to close socket during connection failure", e2);
			}
		}

		try
		{
			outStream = btSocket.getOutputStream();
			inStream = btSocket.getInputStream(); // 可在TextView里显示
		} catch (IOException e)
		{
			Log.e(TAG, "ON RESUME: Output stream creation failed.", e);
		}
		try
		{
			cli.reset_connection(btSocket);
			float v = cli.read_float("rP1");
			String out = String.format("rP1=%f", v);
			Log.e("", out);
			tv.setText(out);

			int pos = 0;
			while (true)
			{
				String id = cli.enum_id(pos);
				v = cli.enum_float(pos);

				if (id == null)
					break;

				if (v != Float.POSITIVE_INFINITY && id != null)
				{
					String log = String.format("%s=%f", id, v);
					pos++;
					Log.e(log, log);
				}
			}
		} catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		tv = (TextView) this.findViewById(R.id.tv_hello);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu)
	{
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);

		menu.findItem(R.id.connect).setOnMenuItemClickListener(new OnMenuItemClickListener()
		{
			public boolean onMenuItemClick(MenuItem item)
			{
				connect();
				return true;
			}
		});
		return true;
	}

}
