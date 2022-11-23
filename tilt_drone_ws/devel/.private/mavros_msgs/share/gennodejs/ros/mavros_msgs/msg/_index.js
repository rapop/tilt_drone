
"use strict";

let VFR_HUD = require('./VFR_HUD.js');
let State = require('./State.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let ParamValue = require('./ParamValue.js');
let LogData = require('./LogData.js');
let FileEntry = require('./FileEntry.js');
let RadioStatus = require('./RadioStatus.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let HilControls = require('./HilControls.js');
let RCIn = require('./RCIn.js');
let PositionTarget = require('./PositionTarget.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let BatteryStatus = require('./BatteryStatus.js');
let HilGPS = require('./HilGPS.js');
let ExtendedState = require('./ExtendedState.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let Thrust = require('./Thrust.js');
let WaypointList = require('./WaypointList.js');
let Vibration = require('./Vibration.js');
let RTCM = require('./RTCM.js');
let DebugValue = require('./DebugValue.js');
let RCOut = require('./RCOut.js');
let Trajectory = require('./Trajectory.js');
let ActuatorControl = require('./ActuatorControl.js');
let LogEntry = require('./LogEntry.js');
let ManualControl = require('./ManualControl.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let HilSensor = require('./HilSensor.js');
let Altitude = require('./Altitude.js');
let Mavlink = require('./Mavlink.js');
let StatusText = require('./StatusText.js');
let HomePosition = require('./HomePosition.js');
let WaypointReached = require('./WaypointReached.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Waypoint = require('./Waypoint.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CommandCode = require('./CommandCode.js');

module.exports = {
  VFR_HUD: VFR_HUD,
  State: State,
  HilStateQuaternion: HilStateQuaternion,
  HilActuatorControls: HilActuatorControls,
  ParamValue: ParamValue,
  LogData: LogData,
  FileEntry: FileEntry,
  RadioStatus: RadioStatus,
  OverrideRCIn: OverrideRCIn,
  HilControls: HilControls,
  RCIn: RCIn,
  PositionTarget: PositionTarget,
  AttitudeTarget: AttitudeTarget,
  BatteryStatus: BatteryStatus,
  HilGPS: HilGPS,
  ExtendedState: ExtendedState,
  GlobalPositionTarget: GlobalPositionTarget,
  CamIMUStamp: CamIMUStamp,
  Thrust: Thrust,
  WaypointList: WaypointList,
  Vibration: Vibration,
  RTCM: RTCM,
  DebugValue: DebugValue,
  RCOut: RCOut,
  Trajectory: Trajectory,
  ActuatorControl: ActuatorControl,
  LogEntry: LogEntry,
  ManualControl: ManualControl,
  TimesyncStatus: TimesyncStatus,
  HilSensor: HilSensor,
  Altitude: Altitude,
  Mavlink: Mavlink,
  StatusText: StatusText,
  HomePosition: HomePosition,
  WaypointReached: WaypointReached,
  OpticalFlowRad: OpticalFlowRad,
  Waypoint: Waypoint,
  ADSBVehicle: ADSBVehicle,
  CommandCode: CommandCode,
};
