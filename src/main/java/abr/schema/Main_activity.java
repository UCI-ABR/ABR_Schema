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
import java.util.ArrayList;
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
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
	public String myFilename;
	
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
	ArrayList<Point> waypoints;
	int flavor;

	//variables for threading
	boolean isDone = false;
	boolean isRunning = false;

	//variables for scanning
	int frameNum = 0;

	//variables for saving position
	FileOutputStream fos;

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
					Calendar calendar = Calendar.getInstance();
					java.util.Date now = calendar.getTime();
					java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
					myFilename = currentTimestamp.toString()+"_SchemaA_moved.csv";
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
					waypoints = new ArrayList<Point>();
					for (int i = 0; i<5; i++) {
						for (int j = 0; j<5; j++) {
							if (i % 2 == 0) {
								waypoints.add(new Point((double)(i*60),(double)(j*60)));
							} else {
								waypoints.add(new Point((double)(i*60),(double)((4-j)*60)));
							}
						}
					}

					dest_x = waypoints.get(0).x;
					dest_y = waypoints.get(0).y;
					Log.i("desty",dest_x+","+dest_y);
					waypoints.remove(0);

					isDone = false;
					isRunning = false;
					Runnable moveThread = new Runnable(){
						public void run() {
							while (true) {
								if (isRunning) {
									if(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10){
										Log.i("haha","obstacle");
										m_ioio_thread.turn(1500);
										m_ioio_thread.counter_left = 0;
										while (m_ioio_thread.counter_left < (int) (4.46 * 13)) {
											m_ioio_thread.move(1400);
										}
										m_ioio_thread.counter_left = 0;
										m_ioio_thread.move(1500);
										while (m_ioio_thread.counter_left < 518/2) { // 525
											m_ioio_thread.turn(1300);
										}
										m_ioio_thread.move(1600);
										m_ioio_thread.turn(1500);
										/*
										while(!isDone){

											if(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10) {
												isDone = true;
												Log.i("haha","isDone");
											}
										}
										*/
									}
									// added a while loop to move forward to adjust robot center after scanning the QR code
									m_ioio_thread.turn(1500);
									rotateOrMove("forward",1600,13);  // 15;13
									m_ioio_thread.move(1500);
									m_ioio_thread.turn(1500);
									//calculate desired angle
									double vectorX = dest_x - curr_x; // horizontal view
									double vectorY = dest_y - curr_y; // horizontal view
									double desired_angle = Math.atan2(vectorY, vectorX) * 180 / Math.PI; //beta
									Log.i("hahaha", "desired_angle:" + desired_angle);
									double angle_to_turn = desired_angle - curr_angle;
									if (angle_to_turn > 180) {angle_to_turn -= 360;}
									else if (angle_to_turn < -180) {angle_to_turn += 360;}
									Log.i("hahaha", "angle2turn:" + angle_to_turn);
									if (angle_to_turn < 0) {
										rotateOrMove("right",1700,Math.abs(angle_to_turn));
										m_ioio_thread.turn(1500);
									} else {
										rotateOrMove("left",1250,Math.abs(angle_to_turn));
										m_ioio_thread.turn(1500);
									}
									//calculate distance
									double desired_distance = Math.sqrt(Math.pow(vectorX, 2) + Math.pow(vectorY, 2));
									Log.i("hahaha", "desired_distance:" + desired_distance);
									m_ioio_thread.counter_left = 0;
									while (m_ioio_thread.counter_left < ((int)(4.46 * (desired_distance-10)))
											&& !isDone && !(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10)) { // desired_distance - 13
										double updated_x = curr_x + (double) m_ioio_thread.counter_left/4.46 * Math.cos(desired_angle); //4.46
										double updated_y = curr_y + (double) m_ioio_thread.counter_left/4.46 * Math.sin(desired_angle); //4.46
										positionLogNSave(myFilename,updated_x, updated_y,(double) m_ioio_thread.counter_left/4.46,
												curr_x,curr_y,dest_x,dest_y, curr_angle, desired_angle, angle_to_turn, 0);
										m_ioio_thread.move(1600);
										//Log.i("hahaha", "moving");
									}
									pause();
									//sweep left
									for(int i = 0; i < 2; i++) {
										rotateOrMove("left",1300,22.5);
										pause();
									}
									//sweep right
									for(int i = 0; i < 4; i++) {
										rotateOrMove("right",1700,22.5);
										pause();
									}
									//sweep left
									for(int i = 0; i < 2; i++) {
										rotateOrMove("left",1300,22.5);
										pause();
									}
									//go backward
									rotateOrMove("backward",1400,10);
									pause();
									//sweep left
									for(int i = 0; i < 2; i++) {
										rotateOrMove("left",1300,22.5);
										pause();
									}
									//sweep right
									for(int i = 0; i < 4; i++) {
										rotateOrMove("right",1700,22.5);
										pause();
									}
									//sweep left
									for(int i = 0; i < 2; i++) {
										rotateOrMove("left",1300,22.5);
										pause();
									}

									while(!isDone && !(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10)) {
										ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
										toneG.startTone(ToneGenerator.TONE_CDMA_ANSWER, 200);
										//m_ioio_thread.move(1600);
										//m_ioio_thread.turn(1500);

										//sweep random direction
										boolean randDir = Math.random() > .5;
										int randAngle = 15 + (int)(Math.random()*(180-15));
										for(int i = 0; i < (int)(Math.ceil(randAngle/22.5)); i++) {
											if (randDir){
												rotateOrMove("left",1300,22.5);
											} else {
												rotateOrMove("right",1700,22.5);
											}
											pause();
										}
										int randDist = 20 + (int)(Math.random()*80);
										for(int i = 0; i < (int)(Math.ceil(randDist/15)); i++) {
											rotateOrMove("forward",1600,15);
											pause();
										}

										/*
										//sweep right with random angle
										int randAngle = 15 + (int)(Math.random()*(180-15));
										rotateOrMove("right",1700,(double)randAngle);
										pause();
										//sweep left with random angle
										randAngle = 15 + (int)(Math.random()*(180-15));
										rotateOrMove("left",1300,(double)randAngle);
										pause();
										//go forward with random distance
										int randDist = 20 + (int)(Math.random()*80);
										rotateOrMove("forward",1600,(double)randDist);
										pause();
										*/
									}
									m_ioio_thread.move(1500);
									m_ioio_thread.turn(1500);
									isDone = false;
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

	public void rotateOrMove(String action, int speed, double amount){
		double encoderValue;
		boolean toTurn;
		if (action.equals("left")){
			encoderValue = 518 / 360.0;  // 525 / 360.0 // 490 / 360.0
			toTurn = true;
		} else if (action.equals("right")) {
			encoderValue = 450 / 360.0;  // 454 / 360.0
			toTurn = true;
		} else if (action.equals("forward") || action.equals("backward")) {
			encoderValue = 4.46;
			toTurn = false;
		} else {
			encoderValue = 0;
			toTurn = false;
		}
		m_ioio_thread.counter_left = 0;
		while (m_ioio_thread.counter_left < (int)(encoderValue * amount) && !isDone
				&& !(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10)) {
			if (toTurn){
				m_ioio_thread.turn(speed);
			} else {
				m_ioio_thread.move(speed);
			}
		}
	}

	public void pause(){
		long startTime = System.currentTimeMillis();
		while(System.currentTimeMillis()-startTime < 1000 && !isDone
				&& !(m_ioio_thread.get_ir2_reading() <= 10 || m_ioio_thread.get_ir1_reading() <= 10 || m_ioio_thread.get_ir3_reading() <= 10)) {
			m_ioio_thread.move(1500);
			m_ioio_thread.turn(1500);
		}
	}

	//Scan for QR code and save information to phone
	public String scan(Mat orig_frame) {
		//Mat frame = new Mat();
		Mat frame = orig_frame.submat(new Rect(0,0,(int)((int)(.75*orig_frame.width())),orig_frame.height()));
		Imgproc.pyrDown(orig_frame, frame);
		//Imgproc.pyrDown(frame, frame);
		Bitmap bMap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(frame, bMap);
		int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];  
		//copy pixel data from the Bitmap into the 'intArray' array  
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());  

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(),intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeReader();     
	    
		String text;
		double aimed_angle;
		
	    try {
			Result result = reader.decode(bitmap);
			text = result.getText();
			ResultPoint[] pts  = result.getResultPoints();
			Log.i("hahaha","middle:"+pts[1].getX() + "," + pts[1].getY());
			//double vectorX = -1*(pts[1].getX() - pts[0].getX()); // horizontal view
			//double vectorY = pts[0].getY() - pts[1].getY(); // horizontal view
			double vectorX = -1*(pts[0].getY() - pts[1].getY()); // vertical view
			double vectorY = pts[0].getX() - pts[1].getX(); // vertical view
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
			flavor = 0;
			if (words.length > 2){
				flavor = Integer.parseInt(words[2].substring(2));
				Log.i("hahaha","curr_flavor:"+flavor);
			}

			if(autoMode) {
				if (!(curr_x == dest_x && curr_y == dest_y) && !(curr_x == old_x && curr_y == old_y)) {
					aimed_angle = Math.atan2(dest_y - curr_y, dest_x - curr_x) * 180 / Math.PI;
					positionLogNSave(myFilename,curr_x, curr_y, 0, curr_x, curr_y, dest_x, dest_y, curr_angle, aimed_angle, aimed_angle-curr_angle, flavor);
					isRunning = true;
					isDone = true;
				}
				else if (curr_x == dest_x && curr_y == dest_y) {
					if(waypoints.isEmpty()) {
						positionLogNSave(myFilename,curr_x, curr_y, 0, curr_x, curr_y, dest_x, dest_y, curr_angle, 0, 0, flavor);
						isDone = true;
						isRunning = false;
					} else {
						dest_x = waypoints.get(0).x;
						dest_y = waypoints.get(0).y;
						waypoints.remove(0);

						aimed_angle = Math.atan2(dest_y - curr_y, dest_x - curr_x) * 180 / Math.PI;
						positionLogNSave(myFilename,curr_x, curr_y, 0, curr_x, curr_y, dest_x, dest_y, curr_angle, aimed_angle, aimed_angle-curr_angle, flavor);

						isRunning = true;
						isDone = true;
					}
				}
				flavor = 0;
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
	public void positionLogNSave(String filename,double x, double y, double distance,
								 double prev_qr_x, double prev_qr_y, double dest_qr_x, double dest_qr_y,
								 double prev_angle, double aimed_angle, double turning_angle, int flavor) {
		double final_x = Math.round(x * 10.0) / 10.0;
		double final_y = Math.round(y * 10.0) / 10.0;
		double final_distance = Math.round(distance * 100.0) / 100.0;
		double final_prev_angle = Math.round(prev_angle * 100.0) / 100.0;
		double final_aimed_angle = Math.round(aimed_angle * 100.0) / 100.0;
		double final_turning_angle = Math.round(turning_angle * 100.0) / 100.0;
		/*
		Calendar calendar = Calendar.getInstance();
		java.util.Date now = calendar.getTime();
		java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
		String time = currentTimestamp.toString();
		*/
		long currentTimeMS = System.currentTimeMillis();
		String time = Long.toString(currentTimeMS);
		String naming = "time,updated_x,updated_y,prev_qr_x,prev_qr_y,dest_qr_x,dest_qr_y,"
				+"distance_from_prev_qr,curr_angle,desired_angle,angle_to_turn,flavor\n";
		String info = time+","+String.valueOf(final_x)+","+String.valueOf(final_y)+","
				+String.valueOf(prev_qr_x)+","+String.valueOf(prev_qr_y)+","+String.valueOf(dest_qr_x)+","
				+String.valueOf(dest_qr_y)+","+String.valueOf(final_distance)+","+String.valueOf(final_prev_angle)+","
				+String.valueOf(final_aimed_angle)+","+String.valueOf(final_turning_angle)+","+String.valueOf(flavor)+"\n";
		try {
			File root = new File(Environment.getExternalStorageDirectory(), "Schema");
			if (!root.exists()) {
				root.mkdirs();
			}
			try {
				File file = new File(root, filename);
				if (!file.exists()) {
					file.createNewFile();
					fos=new FileOutputStream(file,true); // make sure the mode allows appending material to the file
					byte[] n = naming.getBytes();
					fos.write(n);
				}
				try {
					byte[] b = info.getBytes();
					fos.write(b);
					if (!autoMode && file.exists()){
						fos.close();
					}
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

		// added to convert frame to grayscale and enhance contrast
		/*
		Mat frame = new Mat();
		Imgproc.cvtColor(mRgba,frame,Imgproc.COLOR_RGB2GRAY);
		double alpha = 1.0;
		double beta = -5.0;
		frame.convertTo(frame,-1,alpha,beta);
		double thresh = 135;
		double maxVal = 255;
		Imgproc.threshold(frame,frame,thresh,maxVal,Imgproc.THRESH_BINARY);
		*/

		// added for vertical orientation of camera view
		/*
		Mat mRgbaT = mRgba.t();
		Core.flip(mRgba.t(), mRgbaT, 1);
		Imgproc.resize(mRgbaT, mRgbaT, mRgba.size());
		*/
		/*
		mDetector.process(mRgba);
		
		List<MatOfPoint> contours = mDetector.getContours();
		// Log.e(TAG, "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);
		*/
		if (autoMode) { // only move if autoMode is on
			setText("ir1:"+m_ioio_thread.get_ir1_reading(),sonar1Text);
			setText("ir2:"+m_ioio_thread.get_ir2_reading(),sonar2Text);
			setText("ir3:"+m_ioio_thread.get_ir3_reading(),sonar3Text);
			Log.i("hahaha","scanning");
			scan(mRgba);
			/*
			if(frameNum==0)
				scan(mRgba);
			if(frameNum >= 2)
				frameNum = 0;
			else
				frameNum++;
			*/

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
