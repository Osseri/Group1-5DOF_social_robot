package com.robocare.dementia.util;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class SpeakUtility extends AbstractNodeMain {

	private static SpeakUtility __instance = new SpeakUtility();
	public static SpeakUtility getInstance() {
		return __instance;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("SpeakUtility");
	}
	
	private final int __TTS_MAKE_TIMEOUT = 30;
	private final int __SOUND_PLAY_TIMEOUT = 600;
	private boolean __IS_TTS_MAKE_FINISHED = false;
	private boolean __IS_SOUND_PLAY_FINISHED = false;
	
	private final String __TEMP_TTS_FILE = "/opt/robocare/tmp/temp_tts.wav";
	private ServiceClient<robocare_msgs.TTSMakeRequest, robocare_msgs.TTSMakeResponse> __tts_make = null;
	private ServiceClient<robocare_msgs.SoundPlayRequest, robocare_msgs.SoundPlayResponse> __sound_play = null;
	
	@Override
	public void onStart( ConnectedNode connectedNode) {
		try {
			__tts_make = connectedNode.newServiceClient("/robocare_tts/make", robocare_msgs.TTSMake._TYPE);
			__sound_play = connectedNode.newServiceClient("/mini_sound/play", robocare_msgs.SoundPlay._TYPE);
		} catch (ServiceNotFoundException e) {
			e.printStackTrace();
		}
	}
	
	
	
	public void speak(String sentence) {
		makeTTS(sentence);
		playSound(__TEMP_TTS_FILE);
	}
	
	
	private void makeTTS(String sentence) {
		int wait_index = 0;
		__IS_TTS_MAKE_FINISHED = false;
		
		robocare_msgs.TTSMakeRequest request = __tts_make.newMessage();
		request.setFilepath(__TEMP_TTS_FILE);
		request.setText(sentence);
		
		__tts_make.call(request,  new ServiceResponseListener<robocare_msgs.TTSMakeResponse>() {
			@Override
			public void onSuccess(robocare_msgs.TTSMakeResponse response) {
				if( response.getTtsResult() == response.TTS_RESULT_SUCCESS ) {
					// make finish
					__IS_TTS_MAKE_FINISHED = true;
				} else {
					//TODO
				}
			}
			@Override
			public void onFailure(RemoteException arg0) {
				//TODO
			}
		} );
		
		while( !__IS_TTS_MAKE_FINISHED && (wait_index<__TTS_MAKE_TIMEOUT) ) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			wait_index++;
		}
	}
	
	private void playSound(String path) {
		
		__IS_SOUND_PLAY_FINISHED = false;
		int wait_index = 0;
		
		robocare_msgs.SoundPlayRequest request = __sound_play.newMessage();
		request.setFilepath(path);
		
		__sound_play.call(request, new ServiceResponseListener<robocare_msgs.SoundPlayResponse>() {
			@Override
			public void onFailure(RemoteException arg0) {
				// TODO Auto-generated method stub
			}
			@Override
			public void onSuccess(robocare_msgs.SoundPlayResponse response) {
				// TODO Auto-generated method stub
				if(response.getSoundResult() == response.SOUND_RESULT_DONE) {
					__IS_SOUND_PLAY_FINISHED = true;
				} else {
					//TODO
				}
			}
		});
		while( !__IS_SOUND_PLAY_FINISHED && (wait_index<__SOUND_PLAY_TIMEOUT) ) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			wait_index++;
		}
	}
	
}
