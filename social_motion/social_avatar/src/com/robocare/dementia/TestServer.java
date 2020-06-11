package com.robocare.dementia;

import com.robocare.dementia.room.TestRoom;
import com.robocare.dementia.util.SpeakUtility;
import com.robocare.front.server.GameServer;
import com.robocare.front.server.room.RoomContainer;
import com.robocare.rosjava.RosNodeExecutor;

public class TestServer {

	public void init() {
		
		RosNodeExecutor.execute(SpeakUtility.getInstance());
		
		
		new Thread(new Runnable()
		{
			@Override
			public void run()
			{
				GameServer server = new GameServer();
				RoomContainer.getInstance().addRoom("DEMENTIA_MARKET", new TestRoom());
				server.start();

			}
		}).start();
		
		try {
			Thread.sleep(3000L);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		RoomContainer.getInstance().changeCurrentRoom("DEMENTIA_MARKET");
		System.out.println("---START---");
	}
	
	public static void main(String[] args) {
		new TestServer().init();
	}
}
