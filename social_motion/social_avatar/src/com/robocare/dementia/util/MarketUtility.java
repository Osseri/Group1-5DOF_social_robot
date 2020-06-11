package com.robocare.dementia.util;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class MarketUtility {

	private final String REBOOT_DATA_PATH = "system/data/market/";
	private final String REBOOT_DATA_NAME = "reboot_data.txt";
	
	private static MarketUtility __instance = new MarketUtility();
	public static MarketUtility getInstance() {
		return __instance;
	}
	
	public void saveRebootData(MarketRebootData data) {
		
		File myPath = new File(REBOOT_DATA_PATH);
		if(!myPath.exists()) {
			try {
				System.out.println(" >>> create new data path");
				myPath.mkdirs();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		File myFile = new File(REBOOT_DATA_PATH+REBOOT_DATA_NAME);
		if(!myFile.exists()) {
			try {
				System.out.println(" >>> create new data file");
				myFile.createNewFile();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
				
		try	{
			FileOutputStream fo = new FileOutputStream(REBOOT_DATA_PATH+REBOOT_DATA_NAME);
			OutputStreamWriter out = new OutputStreamWriter(fo, "utf-8");
			BufferedWriter bufferedWriter = new BufferedWriter( out );
			bufferedWriter.write(data.getLevel());
			bufferedWriter.newLine();
			bufferedWriter.write(data.getLevelScore());
			bufferedWriter.close();
		} catch ( IOException e )	{
			e.printStackTrace();
		}
	}
	
	public MarketRebootData loadRebootData() {
		File myPath = new File(REBOOT_DATA_PATH);
		if(!myPath.exists()) {
			return new MarketRebootData("0", "0,0,0,0,0,0,0,0,0,0");
		}
		File myFile = new File(REBOOT_DATA_PATH+REBOOT_DATA_NAME);
		if(!myFile.exists()) {
			return new MarketRebootData("0", "0,0,0,0,0,0,0,0,0,0");
		}
		try {
			FileInputStream fi = new FileInputStream(REBOOT_DATA_PATH+REBOOT_DATA_NAME);
			InputStreamReader in = new InputStreamReader(fi, "utf-8");
			BufferedReader br = new BufferedReader(in);
			String temp_level = br.readLine();
			String temp_score = br.readLine();
			return new MarketRebootData(temp_level, temp_score);
		} catch(IOException e) {
			e.printStackTrace();
		}
		return new MarketRebootData("0", "0,0,0,0,0,0,0,0,0,0");
	}
	

	public static void main(String[] args) {
//		MarketUtility.getInstance().saveRebootData( new MarketRebootData("4", "71,80,72,0,0,0,0,0,0,0") );
		
		MarketRebootData data = MarketUtility.getInstance().loadRebootData();
		System.out.println(data.getLevel());
		System.out.println(data.getLevelScore());
		
	}
}
