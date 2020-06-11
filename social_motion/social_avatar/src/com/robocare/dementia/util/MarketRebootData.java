package com.robocare.dementia.util;

public class MarketRebootData {
	private String __SAVE_LEVEL = "0";
	private String __SAVE_LEVEL_SCORE = "no score";
	
	public MarketRebootData(String level, String score) {
		__SAVE_LEVEL = level;
		__SAVE_LEVEL_SCORE = score;
	}
	
	public void setLevel(String level) {
		__SAVE_LEVEL = level;
	}
	public void setLevelScore(String level_score) {
		__SAVE_LEVEL_SCORE = level_score;
	}
	
	public String getLevel() {
		return __SAVE_LEVEL;
	}
	public String getLevelScore() {
		return __SAVE_LEVEL_SCORE;
	}
}
