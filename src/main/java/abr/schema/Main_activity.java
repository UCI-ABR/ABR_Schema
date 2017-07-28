/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.schema;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Calendar;
import java.util.List;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Environment;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class Main_activity extends Activity implements IOIOLooperProvider,CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity
	
	// ioio variables
	IOIO_thread_rover_4wd m_ioio_thread;
	
	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetector;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;
	
	//app state variables
	private boolean autoMode=false;
	
	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravityV;
	float[] mGyro;
	
	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mGravity;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing = 0;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;
	
	//sockets for message passing
	Socket socket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;
	String command = "1500,1500";
	boolean task_state = true;
	String QR = "None";
	long timeSinceLastCommand = System.currentTimeMillis();

	//variables for position
	double curr_x;
	double curr_y;
	double curr_angle;
	double dest_x;
	double dest_y;
	Runnable moveThread;

	//variables for threading
	boolean isDone = false;
	boolean isRunning = false;

	//variables for scanning
	int frameNum = 0;

	//debugging
	final String TAG = "hahaha";

	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		//noinspection StatementWithEmptyBody
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}
	
	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);
		
		helper_.create(); // from IOIOActivity
		
		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		distanceText = (TextView) findViewById(R.id.distanceText);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);

		autoMode = false;

		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					task_state = true;
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;

					//set up sockets for communication with other robots
					/*
					try {
						Object[] objects = (new FileClientAsyncTask()).execute().get();
						socket = (Socket) objects[0];
						dataOutputStream = (DataOutputStream) objects[1];
						dataInputStream = (DataInputStream) objects[2];
					} catch(Exception e) {
						Log.e(TAG, e.getMessage());
					}
					Runnable readThread = new Runnable() {
						public void run(){
							while(task_state) {
								Log.i("command","task state");
								//sendString(QR);
								try {
									//int size = (int) dataInputStream.readByte();
									//Log.i("command","size:"+size);
									int size = 9;
									final byte[] buff = new byte[size];
									dataInputStream.readFully(buff);
									timeSinceLastCommand = System.currentTimeMillis();
									command = new String(buff);
									Log.i("command","command:"+command);
								} catch (EOFException e) {
									runOnUiThread(new Runnable() {
										public void run() {
											Toast.makeText(getApplicationContext()
													, "Connection Down"
													, Toast.LENGTH_SHORT).show();
										}
									});
									Log.i("command","eofexception");
									task_state = false;
									finish();
									Log.e(TAG, e.toString());
								} catch (NumberFormatException e) {
									Log.e(TAG, e.toString());
									Log.i("command","numberformatexception");
								} catch (UnknownHostException e) {
									Log.e(TAG, e.toString());
									Log.i("command","unknown host exception");
								} catch (IOException e) {
									Log.e(TAG, e.toString());
									Log.i("command","ioexception");
								}
							}
						}
					};
					new Thread(readThread).start();
					*/
					dest_x = 120;
					dest_y = 60;

					isDone = false;
					isRunning = false;
					Runnable moveThread = new Runnable(){
						public void run() {
							while (true) {
								if (isRunning) {
									//calculate desired angle
									//double vectorX = dest_x - curr_x; // horizontal view
									//double vectorY = dest_y - curr_y; // horizontal view
									double vectorY = dest_x - curr_x; // vertical view
									double vectorX = dest_y - curr_y; // vertical view
									double desired_angle = Math.atan2(vectorY, vectorX) * 180 / Math.PI; //beta
									Log.i("hahaha", "desired_angle:" + desired_angle);
									double angle_to_turn = desired_angle - curr_angle;
									if (angle_to_turn > 180)
										angle_to_turn -= 360;
									else if (angle_to_turn < -180)
										angle_to_turn += 360;
									Log.i("hahaha", "angle2turn:" + angle_to_turn);
									if (!(angle_to_turn > -10 && angle_to_turn < 10)) {
										if (angle_to_turn < 0) {
											Log.i("hahaha", "<0");
											m_ioio_thread.counter_left = 0;
											while (m_ioio_thread.counter_left < (425 * Math.abs(angle_to_turn) / 360) && !isDone) { //454
												Log.i("hahaha", "turning");
												m_ioio_thread.turn(1700);
											}
											m_ioio_thread.turn(1500);
										} else {
											m_ioio_thread.counter_left = 0;
											while (m_ioio_thread.counter_left < (490 * Math.abs(angle_to_turn) / 360) && !isDone) {
												m_ioio_thread.turn(1300);
											}
											m_ioio_thread.turn(1500);
										}
									}
									//calculate distance
									double desired_distance = Math.sqrt(Math.pow(vectorX, 2) + Math.pow(vectorY, 2));
									Log.i("hahaha", "desired_distance:" + desired_distance);
									m_ioio_thread.counter_left = 0;
									while (m_ioio_thread.counter_left < (446 / 100 * desired_distance) && !isDone) {
										m_ioio_thread.move(1600);
									}
									m_ioio_thread.move(1500);
								} else {
									m_ioio_thread.move(1500);
									m_ioio_thread.turn(1500);
								}
							}
						}
					};

					Thread t1 = new Thread(moveThread);
					t1.start();
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
					isRunning = false;
					isDone = true;
					/*
					task_state = false;
					try {
						socket.close();
					} catch (IOException e) {
						Log.e(TAG, e.toString());
					}
					*/
				}
			}
		});

		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}
	}

	//Scan for QR code and save information to phone
	public String scan(Mat orig_frame) {
		Mat frame = new Mat();
		Imgproc.pyrDown(orig_frame, frame);
		Imgproc.pyrDown(frame, frame);
		Bitmap bMap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(frame, bMap);
		int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];  
		//copy pixel data from the Bitmap into the 'intArray' array  
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());  

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(),intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeReader();     
	    
		String text; 
		
	    try {
			Result result = reader.decode(bitmap);
			text = result.getText();
			ResultPoint[] pts  = result.getResultPoints();
			Log.i("hahaha","middle:"+pts[1].getX() + "," + pts[1].getY());
			double vectorX = -1*(pts[1].getX() - pts[0].getX());
			double vectorY = pts[0].getY() - pts[1].getY();
			Log.i("hahaha","vectorY:"+vectorY);
			Log.i("hahaha","vectorX:"+vectorX);
			Log.i("hahaha","angle:"+Math.atan2(vectorY,vectorX)*180/Math.PI);
			curr_angle = Math.atan2(vectorY,vectorX)*180/Math.PI;
			String[] words=text.split(",");

			double old_x = curr_x;
			double old_y = curr_y;


			curr_x = Double.parseDouble(words[0]);
			curr_y = Double.parseDouble(words[1]);

			Log.i("hahaha","curr_loc:"+curr_x+","+curr_y);

			if(autoMode) {
				if (!(curr_x == dest_x && curr_y == dest_y) && !(curr_x == old_x && curr_y == old_y)) {
					positionLogNSave(curr_x, curr_y); // added to log the newly detected QR code position
					// added a while loop to move forward to adjust robot center after scanning the QR code
					while (m_ioio_thread.counter_left < (int) (4.46 * 16)) {
						m_ioio_thread.move(1580);
					}
					m_ioio_thread.move(1500);
					m_ioio_thread.turn(1500);
					Log.i("hahaha", "interrupt");
					isRunning = true;
					isDone = true;
				}
				else if (curr_x == dest_x && curr_y == dest_y) {
					isDone = true;
					isRunning = false;
				}
			}
			/*
			Calendar calendar = Calendar.getInstance();
			java.util.Date now = calendar.getTime();
			java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
			String time = currentTimestamp.toString();
			String info = text;
			try {
			    File newFolder = new File(Environment.getExternalStorageDirectory(), "Schema");
			    if (!newFolder.exists()) {
			        newFolder.mkdir();
			    }
			    try {
			        File file = new File(newFolder, time + ".txt");
			        file.createNewFile();
			        FileOutputStream fos=new FileOutputStream(file);
	                try {
	                	byte[] b = info.getBytes();
	                    fos.write(b);
	                    fos.close();
	                    ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
	        			toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
	                } catch (IOException e) {
	                	Log.e(TAG,"Couldn't write to SD");
	                }
			    } catch (Exception ex) {
			    	Log.e(TAG,"Couldn't write to SD");
			    }
			} catch (Exception e) {
			    Log.e(TAG,"Couldn't write to SD");
			}
			*/
			QR = text;
			Log.i(TAG,text);
			ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
			toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
			return text;
		} catch (NotFoundException e) {
			QR = "None";
			e.printStackTrace();
			text = "no code found";
		} catch (ChecksumException e) {
			QR = "None";
			e.printStackTrace();
			text =  "checksum error";
		} catch (FormatException e) {
			QR = "None";
			e.printStackTrace();
			text = "format error";
		}
	    Log.i(TAG,text);

		return text;
	}

	// to log the current robot position along with the time stamp and append updated info to a file
	public void positionLogNSave(double x, double y) {
		Calendar calendar = Calendar.getInstance();
		java.util.Date now = calendar.getTime();
		java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
		String time = currentTimestamp.toString();
		String info = "Time:" + time + ", Position:(" + String.valueOf(x) + "," + String.valueOf(y) + ")";
		try {
			File root = new File(Environment.getExternalStorageDirectory(), "Schema");
			if (!root.exists()) {
				root.mkdirs();
			}
			try {
				File file = new File(root, "Schema_Position.txt");
				if (!file.exists()) {
					file.createNewFile();
				}
				FileOutputStream fos=new FileOutputStream(file,true); // make sure the mode allows appending material to the file
				try {
					byte[] b = info.getBytes();
					fos.write(b);
					fos.close();
				} catch (IOException e) {
					Log.e("app.main","Couldn't write to SD");
				}
			} catch (Exception ex) {
				Log.e("app.main","Couldn't write to SD");
			}
		} catch (Exception e) {
			Log.e("app.main","Couldn't write to SD");
		}
		Log.i("Saved",info);
	}

	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
		task_state = true;
	    if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
	}
	
	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		task_state = false;
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}
	
	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		//To set color, find HSV values of desired color and convert each value to 1-255 scale
		//mDetector.setHsvColor(new Scalar(7, 196, 144)); // red
		mDetector.setHsvColor(new Scalar(253.796875,222.6875,195.21875));
	}
	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		mRgba = inputFrame.rgba();

		// added for vertical orientation of camera view
		/*
		Mat mRgbaT = mRgba.t();
		Core.flip(mRgba.t(), mRgbaT, 1);
		Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
		*/

		mDetector.process(mRgba);
		
		List<MatOfPoint> contours = mDetector.getContours();
		// Log.e(TAG, "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);
		if (autoMode) { // only move if autoMode is on
			Log.i("hahaha","scanning");
			if(frameNum==0)
				scan(mRgba);
			if(frameNum >= 5)
				frameNum = 0;
			else
				frameNum++;
			/*
			sendString(QR+",");
			if(System.currentTimeMillis()-timeSinceLastCommand < 100){
				String[] commands = command.split(",");
				int speedCommand = Integer.parseInt(commands[0]);
				int steeringCommand = Integer.parseInt(commands[1]);
				Log.i("command","speedCommand:"+speedCommand);
				Log.i("command","steeringCommand:"+steeringCommand);
				m_ioio_thread.move(speedCommand);
				m_ioio_thread.turn(steeringCommand);
			} else {
				m_ioio_thread.move(1500);
				m_ioio_thread.turn(1500);
			}
			*/
		}

		return mRgba;
	}
	
	//send an integer using output stream from socket
	public void sendString(String str){
		if(dataOutputStream != null)
			try {
				//dataOutputStream.writeInt(str.length());
				dataOutputStream.write(str.getBytes());
				dataOutputStream.flush();
				Log.i(command, "QR sent");
			} catch (IOException e) {
				Log.e(command,e.getMessage());
			}
	}

	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;
		  
	}
	
	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}
	
	//set the text of any text view in this application
	public void setText(final String str, final TextView tv) 
	{
		  runOnUiThread(new Runnable() {
			  @Override
			  public void run() {
				  tv.setText(str);
			  }
		  });
	}

	/****************************************************** functions from IOIOActivity *********************************************************************************/
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
						.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread_rover_4wd();
			return m_ioio_thread;
		} else
			return null;
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		helper_.start();
	}

	@Override
	protected void onStop() {
		super.onStop();
		helper_.stop();
		try {
			if(socket != null)
				socket.close();
		} catch (IOException e) {
			Log.e(TAG, e.getMessage());
		}
		
	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
			if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}
}
