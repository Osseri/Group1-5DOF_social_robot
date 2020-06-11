package com.robocare.dementia.room;

import com.robocare.dementia.model.DEMENTIA_CONSTANT;
import com.robocare.dementia.util.MarketRebootData;
import com.robocare.dementia.util.MarketUtility;
import com.robocare.dementia.util.ShellUtility;
import com.robocare.dementia.util.SpeakUtility;
import com.robocare.front.message.GameMessage;
import com.robocare.front.server.room.Room;
import com.robocare.rosjava.util.listener.UtilityMessageListener;
import com.robocare.rosjava.util.listener.UtilityMessageListenerFactory;

public class TestRoom extends Room implements UtilityMessageListener{

	private boolean _DO_SPEAK = false;
	private boolean _IS_SPEAK_STOPPED = false;
	private int _SPEAK_DURATION = 5000; 
	
	public TestRoom() {
		super("DEMENTIA_MARKET");
	}
	
	public void ready() {
	}
	
	public void end() {
		UtilityMessageListenerFactory.getInstance().removeListener(this);
	}

	@Override
	public void service(GameMessage reqMessage, GameMessage resMessage) {

		System.out.println("--------------------");
		System.out.println(reqMessage.toXML());
		System.out.println("--------------------");
		
		String opString = reqMessage.getOperationName();
		
		try {
			System.out.println("[receive] msg name : " + opString);
			TestOperationName op = TestOperationName.valueOf(opString);
			switch(op) {
			case req_play_voice :
				resMessage.setOperationName(TestOperationName.res_play_voice.toString());
				String sentence = reqMessage.getParameterValue("sentence");
				doPlayVoice(sentence);
				break;
			
			case req_stop_voice :
				resMessage.setOperationName(TestOperationName.res_stop_voice.toString());
				_IS_SPEAK_STOPPED = true;
				break;
				
			case req_game_end :
				resMessage.setOperationName(TestOperationName.res_game_end.toString());
				ShellUtility.getInstance().exitGame();
				break;
				
			case req_game_state :
				resMessage.setOperationName(TestOperationName.res_game_state.toString());
				String my_state = reqMessage.getParameterValue("state");
				System.out.println("\t state : " + my_state);
				break;
				
			case req_game_result :
				resMessage.setOperationName(TestOperationName.res_game_result.toString());
				break;
			case req_save_date :
				resMessage.setOperationName(TestOperationName.res_save_date.toString());
				String save_level = reqMessage.getParameterValue("save_level");
				String first_quiz = reqMessage.getParameterValue("first_quiz");
				String second_quiz = reqMessage.getParameterValue("second_quiz");
				System.out.println("\t save_level : " + save_level);
				System.out.println("\t first_quiz : " + first_quiz);
				System.out.println("\t second_quiz : " + second_quiz);
				break;
				
			case req_save_rebootdata :
				resMessage.setOperationName(TestOperationName.res_save_rebootdata.toString());
				String reboot_save_level = reqMessage.getParameterValue("save_level");
				String reboot_save_score = reqMessage.getParameterValue("save_level_score");
				MarketUtility.getInstance().saveRebootData(new MarketRebootData(reboot_save_level, reboot_save_score));
				break;
				
			case req_load_rebootdata :
				resMessage.setOperationName(TestOperationName.res_load_rebootdata.toString());
				MarketRebootData reboot_data = MarketUtility.getInstance().loadRebootData();
				resMessage.addParameter("save_level", reboot_data.getLevel());
				resMessage.addParameter("save_level_score", reboot_data.getLevelScore());
				break;
				
			case test_noti_play_voice_finished :
				resMessage.setOperationName(TestOperationName.res_test_noti.toString());
				GameMessage test_noti_play_voice_finished = new GameMessage();
				test_noti_play_voice_finished.setOperationName(TestOperationName.noti_play_voice_finished.toString());
				System.out.println("[send] " + test_noti_play_voice_finished.getOperationName() + "\n ");
				broadcast(test_noti_play_voice_finished);
				break;
				
			case test_noti_unknown_message :
				resMessage.setOperationName(TestOperationName.res_test_noti.toString());
				GameMessage test_noti_unknown_message = new GameMessage();
				test_noti_unknown_message.setOperationName(TestOperationName.noti_unknown_message.toString());
				System.out.println("[send] " + test_noti_unknown_message.getOperationName() + "\n ");
				broadcast(test_noti_unknown_message);
				break;
				
			default:
				resMessage.setOperationName(TestOperationName.res_fail.toString());
				GameMessage message = new GameMessage();
				message.setOperationName(TestOperationName.noti_unknown_message.toString());
				System.out.println("[send] " + message.getOperationName() + "\n ");
				broadcast(message);
				break;
			}
			System.out.println("[send] " + resMessage.getOperationName() + "\n ");
		} catch(Exception e) {
			e.printStackTrace();
		}
	}

	private void doPlayVoice(final String sentence) {
		
		if(DEMENTIA_CONSTANT.USE_TTS) {
			
			SpeakUtility.getInstance().speak(sentence);
			
			GameMessage message = new GameMessage();
			message.setOperationName(TestOperationName.noti_play_voice_finished.toString());
			System.out.println("[send] " + message.getOperationName() + "\n ");
			broadcast(message);
			
		} else {
			_DO_SPEAK = true;
			new Thread( new Runnable() {
				@Override
				public void run() {
					System.out.println("[play] sentence : " + sentence);
					int index = 0;
					int loop_sleep = 100;
					
					while( (index*loop_sleep<_SPEAK_DURATION) && (!_IS_SPEAK_STOPPED)) {
						try {
							Thread.sleep(loop_sleep);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
						index++;
					}
					GameMessage message = new GameMessage();
					message.setOperationName(TestOperationName.noti_play_voice_finished.toString());
					System.out.println("[send] " + message.getOperationName() + "\n ");
					broadcast(message);
					_DO_SPEAK = false;
					_IS_SPEAK_STOPPED = false;
				}
			} ).start();	
		}
	}
	
	@Override
	public void handleMessage(String msg) {
		
	}

}