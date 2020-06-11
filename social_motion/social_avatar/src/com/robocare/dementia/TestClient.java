package com.robocare.dementia;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.robocare.front.ui.ExtraUIData;
import com.robocare.front.ui.GameExtraPanel;
import com.robocare.front.ui.GameExtraUI;

public class TestClient {

	public void init() {
		String name = "TestUI";
		Map<String, GameExtraPanel> messagePanelMap = new HashMap<String, GameExtraPanel>();
		
		List<ExtraUIData> mainMenuDataList = new ArrayList<ExtraUIData>();
				
		ExtraUIData req_play_voice = new ExtraUIData("req_play_voice", "req_play_voice");
		req_play_voice.addParam("sentence");
		mainMenuDataList.add(req_play_voice);
		
		ExtraUIData req_stop_voice = new ExtraUIData("req_stop_voice", "req_stop_voice");
		mainMenuDataList.add(req_stop_voice);
		
		ExtraUIData req_game_end = new ExtraUIData("req_game_end", "req_game_end");
		mainMenuDataList.add(req_game_end);
		
		ExtraUIData req_game_state = new ExtraUIData("req_game_state", "req_game_state");
		mainMenuDataList.add(req_game_state);
		
		ExtraUIData req_game_result = new ExtraUIData("req_game_result", "req_game_result");
		mainMenuDataList.add(req_game_result);
		
		ExtraUIData req_save_data = new ExtraUIData("req_save_data", "req_save_data");
		req_save_data.addParam("save_level");
		req_save_data.addParam("first_quiz");
		req_save_data.addParam("second_quiz");
		req_save_data.addParam("first_quiz-hint");
		req_save_data.addParam("second_quiz-hint");
		mainMenuDataList.add(req_save_data);
		
		ExtraUIData req_save_rebootdata = new ExtraUIData("req_save_rebootdata", "req_save_rebootdata");
		req_save_rebootdata.addParam("save_level");
		req_save_rebootdata.addParam("save_level_score");
		mainMenuDataList.add(req_save_rebootdata);
		
		ExtraUIData req_load_rebootdata = new ExtraUIData("req_load_rebootdata", "req_load_rebootdata");
		mainMenuDataList.add(req_load_rebootdata);		
		
		ExtraUIData test_noti_play_voice_finished = new ExtraUIData("test_noti_play_voice_finished", "test_noti_play_voice_finished");
		mainMenuDataList.add(test_noti_play_voice_finished);
		
		ExtraUIData test_noti_unknown_message = new ExtraUIData("test_noti_unknown_message", "test_noti_unknown_message");
		mainMenuDataList.add(test_noti_unknown_message);
		
		

		GameExtraPanel mainMenu = new GameExtraPanel(mainMenuDataList);
		messagePanelMap.put("noti_set_image_main_menu", mainMenu);
		
		GameExtraUI ui = new GameExtraUI(name, messagePanelMap);
		
		ui.setInitialPanel(mainMenu);
	}
	
	public static void main(String[] args) {
		new TestClient().init();
	}
}
