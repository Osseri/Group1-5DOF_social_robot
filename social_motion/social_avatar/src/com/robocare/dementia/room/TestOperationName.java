package com.robocare.dementia.room;

public enum TestOperationName {
	
	req_play_voice,		res_play_voice,		noti_play_voice_finished,
	req_stop_voice,		res_stop_voice,
	
	req_game_end,		res_game_end,
	req_game_state,		res_game_state,
	req_game_result,	res_game_result,
	req_save_date,		res_save_date,
	
	req_save_rebootdata,	res_save_rebootdata,
	req_load_rebootdata,	res_load_rebootdata,
	
						res_fail,			noti_unknown_message,

	res_test_noti,					
	test_noti_play_voice_finished,
	test_noti_unknown_message,
}
