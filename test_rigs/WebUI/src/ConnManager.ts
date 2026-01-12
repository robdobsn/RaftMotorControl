import { RaftConnector, RaftEventFn, RaftSystemUtils } from '@robdobsn/raftjs';
import SystemTypeTwoStepper from './types/SystemTypeTwoStepper';

export default class ConnManager {
  private static _instance: ConnManager;
  
  private _connector = new RaftConnector(async (systemUtils: RaftSystemUtils) => {
    return new SystemTypeTwoStepper();
  });
  
  private _onConnectionEvent: RaftEventFn | null = null;

  public static getInstance(): ConnManager {
    if (!ConnManager._instance) {
      ConnManager._instance = new ConnManager();
    }
    return ConnManager._instance;
  }

  public setConnectionEventListener(listener: RaftEventFn): void {
    this._onConnectionEvent = listener;
  }

  public isConnected(): boolean {
    return this._connector.isConnected();
  }

  public getConnector(): RaftConnector {
    return this._connector;
  }

  public async connect(ipAddress: string): Promise<boolean> {
    this._connector.setEventListener((evtType, eventEnum, eventName, eventData) => {
      if (this._onConnectionEvent) {
        this._onConnectionEvent(evtType, eventEnum, eventName, eventData);
      }
    });
    
    await this._connector.initializeChannel('WebSocket');
    return this._connector.connect(ipAddress);
  }

  public disconnect(): Promise<void> {
    return this._connector.disconnect();
  }
}
