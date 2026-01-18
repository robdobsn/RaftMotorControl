import {
  RaftSystemType,
  RaftSystemUtils,
  RaftEventFn,
  RaftDeviceManager,
  RaftSubscribeForUpdatesCBType,
  RaftOKFail,
  RaftLog,
  RaftPublishEvent,
  RaftPublishEventNames
} from '@robdobsn/raftjs';

export default class SystemTypeTwoStepper implements RaftSystemType {
  nameForDialogs = 'Two Stepper Test Rig';
  defaultWiFiHostname = 'TwoStepperRig';
  firmwareDestName = 'ricfw';
  normalFileDestName = 'fs';
  connectorOptions = { wsSuffix: 'ws', bleConnItvlMs: 50 };
  
  private _onEvent: RaftEventFn | null = null;
  private _systemUtils: RaftSystemUtils | null = null;
  private _deviceManager: RaftDeviceManager = new RaftDeviceManager();

  setup(systemUtils: RaftSystemUtils, onEvent: RaftEventFn | null): void {
    this._systemUtils = systemUtils;
    this._onEvent = onEvent;
    this._deviceManager.setup(systemUtils);
  }

  subscribeForUpdates: RaftSubscribeForUpdatesCBType | null = async (
    systemUtils: RaftSystemUtils,
    enable: boolean
  ) => {
    const subscribeRateHz = 10; // 10Hz for motor/encoder data
    try {
      const topic = 'devjson';
      const subscribeDisable = '{"cmdName":"subscription","action":"update",' +
        '"pubRecs":[' +
        `{"name":"${topic}","rateHz":0}` +
        ']}';
      const subscribeEnable = '{"cmdName":"subscription","action":"update",' +
        '"pubRecs":[' +
        `{"name":"${topic}","trigger":"timeorchange","rateHz":${subscribeRateHz.toString()}}` +
        ']}';
      
      const msgHandler = systemUtils.getMsgHandler();
      const ricResp = await msgHandler.sendRICRESTCmdFrame<RaftOKFail>(
        enable ? subscribeEnable : subscribeDisable
      );
      
      RaftLog.debug(`subscribe enable/disable returned ${JSON.stringify(ricResp)}`);
    } catch (error: unknown) {
      RaftLog.warn(`Failed to subscribe for updates ${error}`);
    }
  };

  stateIsInvalid(): void {
    // Invalidate state if needed
  }

  rxOtherMsgType = (payload: Uint8Array, frameTimeMs: number): void => {
    RaftLog.verbose(`rxOtherMsgType payloadLen ${payload.length}`);
    
    // Handle JSON messages (not binary)
    const decoder = new TextDecoder('utf-8');
    const jsonString = decoder.decode(payload.slice(2));
    
    // Handle using device manager
    this._deviceManager.handleClientMsgJson(jsonString);
    
    // Call event handler if registered
    if (this._onEvent) {
      this._onEvent('pub', RaftPublishEvent.PUBLISH_EVENT_DATA,
        RaftPublishEventNames[RaftPublishEvent.PUBLISH_EVENT_DATA],
        {
          topicIDs: [],
          payload: payload,
          frameTimeMs: frameTimeMs,
          isBinary: false
        });
    }
  };

  deviceMgrIF = this._deviceManager;
}
