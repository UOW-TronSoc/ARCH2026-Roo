import { createContext, useContext, useState, useCallback, useRef } from 'react';

const initialTelem = {
  voltage_v: null,
  current_a: null,
  energy_mwh: null,
  pitch_deg: null,
  roll_deg: null,
  enc1_angle: null,
  enc2_angle: null,
  ping_ms: null,
};

const TelemetryContext = createContext(null);

export function TelemetryProvider({ children }) {
  const [mode, setMode] = useState('SIM');
  const [telem, setTelem] = useState(initialTelem);
  const [connected, setConnected] = useState(false);
  const [speedPct, setSpeedPct] = useState(80);
  const [suspLocked, setSuspLocked] = useState(false);
  const [frontCam, setFrontCam] = useState(true);
  const [backCam, setBackCam] = useState(false);
  const [gimbalCam, setGimbalCam] = useState(false);

  const driveRef = useRef({ left: 0, right: 0 });
  const gHAngleRef = useRef(0);
  const gVAngleRef = useRef(0);
  const sFrontTargetRef = useRef(0);
  const sBackTargetRef = useRef(0);
  const sendRef = useRef(null);
  const keyStateRef = useRef({
    w: false, a: false, s: false, d: false,
    ArrowLeft: false, ArrowRight: false, ArrowUp: false, ArrowDown: false,
    u: false, i: false, o: false, p: false,
  });

  const updateTelem = useCallback((updates) => {
    setTelem((prev) => ({ ...prev, ...updates }));
  }, []);

  const resetTaskPower = useCallback(() => {
    if (sendRef.current) {
      sendRef.current({ type: 'command', id: 20, value: 0 });
    }
  }, []);

  const value = {
    mode,
    setMode,
    telem,
    updateTelem,
    connected,
    setConnected,
    speedPct,
    setSpeedPct,
    suspLocked,
    setSuspLocked,
    frontCam,
    setFrontCam,
    backCam,
    setBackCam,
    gimbalCam,
    setGimbalCam,
    driveRef,
    gHAngleRef,
    gVAngleRef,
    sFrontTargetRef,
    sBackTargetRef,
    sendRef,
    keyStateRef,
    resetTaskPower,
  };

  return (
    <TelemetryContext.Provider value={value}>
      {children}
    </TelemetryContext.Provider>
  );
}

export function useTelemetry() {
  const ctx = useContext(TelemetryContext);
  if (!ctx) throw new Error('useTelemetry must be used within TelemetryProvider');
  return ctx;
}
