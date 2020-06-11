package com.robocare.dementia.util;

import java.io.IOException;

public class ShellUtility {

	private static ShellUtility __instance = new ShellUtility();
	public static ShellUtility getInstance() {
		return __instance;
	}
	
	
	public void exitGame() {
		try {
			Thread.sleep(300);
			Runtime.getRuntime().exec("killall flashplayerdebugger");
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
	}
	
	public static void main(String[] args) {
		ShellUtility.getInstance().exitGame();
	}
}
