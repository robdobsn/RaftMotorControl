import { RaftConnector, RaftEventFn, RaftSystemUtils } from '@robdobsn/raftjs';
import SystemTypeTwoStepper from './types/SystemTypeTwoStepper';

export default class ConnManager {
  private static _instance: ConnManager;
  
  // Sensor device connector (for magnetic rotation sensors)
  private _sensorConnector = new RaftConnector(async (systemUtils: RaftSystemUtils) => {
    return new SystemTypeTwoStepper();
  });
  
  // Motor controller connector (may be same as sensor or separate device)
  private _motorConnector = new RaftConnector(async (systemUtils: RaftSystemUtils) => {
    return new SystemTypeTwoStepper();
  });
  
  private _onSensorConnectionEvent: RaftEventFn | null = null;
  private _onMotorConnectionEvent: RaftEventFn | null = null;
  private _useSeparateMotorController: boolean = true;
  private _sensorIpAddress: string = '';

  public static getInstance(): ConnManager {
    if (!ConnManager._instance) {
      ConnManager._instance = new ConnManager();
    }
    return ConnManager._instance;
  }

  public setSensorConnectionEventListener(listener: RaftEventFn): void {
    this._onSensorConnectionEvent = listener;
  }

  public setMotorConnectionEventListener(listener: RaftEventFn): void {
    this._onMotorConnectionEvent = listener;
  }

  public setUseSeparateMotorController(useSeparate: boolean): void {
    this._useSeparateMotorController = useSeparate;
  }

  public getUseSeparateMotorController(): boolean {
    return this._useSeparateMotorController;
  }

  public getSensorIpAddress(): string {
    return this._sensorIpAddress;
  }

  public isSensorConnected(): boolean {
    return this._sensorConnector.isConnected();
  }

  public isMotorConnected(): boolean {
    if (!this._useSeparateMotorController) {
      return this._sensorConnector.isConnected();
    }
    return this._motorConnector.isConnected();
  }

  // Returns the sensor connector (for reading encoder data)
  public getSensorConnector(): RaftConnector {
    return this._sensorConnector;
  }

  // Returns appropriate connector for motor control
  public getMotorConnector(): RaftConnector {
    if (!this._useSeparateMotorController) {
      return this._sensorConnector;
    }
    return this._motorConnector;
  }

  // Legacy method for backward compatibility (returns sensor connector)
  public getConnector(): RaftConnector {
    return this._sensorConnector;
  }

  // Legacy method for backward compatibility
  public isConnected(): boolean {
    return this.isSensorConnected();
  }

  // Legacy method for backward compatibility
  public setConnectionEventListener(listener: RaftEventFn): void {
    this.setSensorConnectionEventListener(listener);
  }

  public async connectSensor(ipAddress: string): Promise<boolean> {
    this._sensorIpAddress = ipAddress;
    this._sensorConnector.setEventListener((evtType, eventEnum, eventName, eventData) => {
      if (this._onSensorConnectionEvent) {
        this._onSensorConnectionEvent(evtType, eventEnum, eventName, eventData);
      }
    });
    
    // Enable auto-reconnect with 60 second retry period
    this._sensorConnector.setRetryConnectionIfLost(true, 60);
    
    await this._sensorConnector.initializeChannel('WebSocket');
    return this._sensorConnector.connect(ipAddress);
  }

  public async connectMotor(ipAddress: string): Promise<boolean> {
    if (!this._useSeparateMotorController) {
      // Use sensor connection
      return this._sensorConnector.isConnected();
    }

    this._motorConnector.setEventListener((evtType, eventEnum, eventName, eventData) => {
      if (this._onMotorConnectionEvent) {
        this._onMotorConnectionEvent(evtType, eventEnum, eventName, eventData);
      }
    });
    
    // Enable auto-reconnect with 60 second retry period
    this._motorConnector.setRetryConnectionIfLost(true, 60);
    
    await this._motorConnector.initializeChannel('WebSocket');
    return this._motorConnector.connect(ipAddress);
  }

  // Legacy method for backward compatibility
  public async connect(ipAddress: string): Promise<boolean> {
    return this.connectSensor(ipAddress);
  }

  public async disconnectSensor(): Promise<void> {
    return this._sensorConnector.disconnect();
  }

  public async disconnectMotor(): Promise<void> {
    if (!this._useSeparateMotorController) {
      return;
    }
    return this._motorConnector.disconnect();
  }

  // Legacy method for backward compatibility
  public disconnect(): Promise<void> {
    return this.disconnectSensor();
  }
}
