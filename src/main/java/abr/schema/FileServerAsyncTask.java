package abr.schema;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

public class FileServerAsyncTask extends AsyncTask<Void,Void,Object[]> {
	Object[] objects = new Object[4];
    @Override
    protected Object[] doInBackground(Void... params) {
        try {
            ServerSocket serverSocket = new ServerSocket(8888);
            objects[0] = serverSocket;
            Log.i("hahaha", "server socket started");
            Log.i("hahaha", "server socket IP:"+serverSocket.getInetAddress());
            Socket clientSocket = serverSocket.accept();
            Log.i("hahaha", clientSocket.getRemoteSocketAddress().toString());
            Log.i("hahaha", clientSocket.getLocalSocketAddress().toString());
            objects[1] = clientSocket;
            Log.i("hahaha", "accepted");
            DataInputStream inputStream = new DataInputStream(clientSocket.getInputStream());
            objects[2] = inputStream;
            DataOutputStream outputStream = new DataOutputStream(clientSocket.getOutputStream());
            objects[3] = outputStream;
            return objects;
        } catch (IOException e) {
            Log.e("hahaha", e.getMessage());
            return null;
        }
    }
}