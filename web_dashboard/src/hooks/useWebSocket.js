import { useEffect, useRef } from 'react';
import { WS_URL } from '../config';
import { useTelemetry } from '../contexts/TelemetryContext';

const MAX_CMD = 10.0;
const SEND_HZ = 10;

function clamp(n, a, b) {
  return Math.min(b, Math.max(a, n));
}

export function useWebSocket() {
  const {
    setMode,
    updateTelem,
    setConnected,
    driveRef,
    gHAngleRef,
    gVAngleRef,
    sendRef,
  } = useTelemetry();

  const wsRef = useRef(null);
  const lastTelemRxRef = useRef(0);
  const reconnectTimeoutRef = useRef(null);

  useEffect(() => {
    function readCommands() {
      const leftNorm = driveRef.current?.left ?? 0;
      const rightNorm = driveRef.current?.right ?? 0;
      const motorLeft = clamp(leftNorm, -1, 1) * MAX_CMD;
      const motorRight = clamp(rightNorm, -1, 1) * MAX_CMD;

      const gH = gHAngleRef.current ?? 0;
      const gV = gVAngleRef.current ?? 0;
      const g2 = clamp(gH + 90, 0, 180);
      const g3 = clamp(gV + 90, 0, 180);

      const suspFront = 0;
      const suspBack = 0;

      return { motor_left: motorLeft, motor_right: motorRight, g2, g3, susp_front: suspFront, susp_back: suspBack };
    }

    function send(obj) {
      try {
        if (wsRef.current?.readyState === 1) {
          wsRef.current.send(JSON.stringify(obj));
        }
      } catch (_) {}
    }

    sendRef.current = send;

    function connect() {
      setConnected(false);
      try {
        wsRef.current = new WebSocket(WS_URL);
      } catch (e) {
        reconnectTimeoutRef.current = setTimeout(connect, 1500);
        return;
      }

      wsRef.current.onopen = () => {
        setConnected(true);
        setMode('CTL');
        send({ type: 'control_source', value: 'web' });
        send({ type: 'stop' });
      };

      wsRef.current.onclose = () => {
        setConnected(false);
        setMode('SIM');
        reconnectTimeoutRef.current = setTimeout(connect, 1500);
      };

      wsRef.current.onerror = () => {
        try {
          wsRef.current?.close();
        } catch (_) {}
      };

      wsRef.current.onmessage = (ev) => {
        let msg;
        try {
          msg = JSON.parse(ev.data);
        } catch (_) {
          return;
        }
        if (!msg || msg.type !== 'telemetry') return;

        const updates = {};
        if (typeof msg.pitch_deg === 'number') updates.pitch_deg = msg.pitch_deg;
        if (typeof msg.roll_deg === 'number') updates.roll_deg = msg.roll_deg;
        if (typeof msg.voltage_v === 'number') updates.voltage_v = msg.voltage_v;
        if (typeof msg.current_a === 'number') updates.current_a = msg.current_a;
        if (typeof msg.energy_mwh === 'number') updates.energy_mwh = msg.energy_mwh;
        if (typeof msg.enc1_angle === 'number') updates.enc1_angle = msg.enc1_angle;
        if (typeof msg.enc2_angle === 'number') updates.enc2_angle = msg.enc2_angle;

        const now = Date.now();
        if (lastTelemRxRef.current > 0) {
          const p = now - lastTelemRxRef.current;
          if (p > 0) updates.ping_ms = p;
        }
        lastTelemRxRef.current = now;

        updateTelem(updates);
      };
    }

    connect();

    const sendInterval = setInterval(() => {
      if (!wsRef.current || wsRef.current.readyState !== 1) return;
      const c = readCommands();
      send({ type: 'motor', left: c.motor_left, right: c.motor_right });
      send({ type: 'gimbal', g2: c.g2, g3: c.g3 });
      send({ type: 'susp', front: c.susp_front, back: c.susp_back });
      send({ type: 'heartbeat' });
    }, Math.max(50, Math.round(1000 / SEND_HZ)));

    const onKeydown = (e) => {
      if (e.code === 'Space') {
        e.preventDefault();
        send({ type: 'stop' });
      }
    };

    window.addEventListener('keydown', onKeydown, { passive: false });

    return () => {
      clearInterval(sendInterval);
      clearTimeout(reconnectTimeoutRef.current);
      window.removeEventListener('keydown', onKeydown);
      try {
        wsRef.current?.close();
      } catch (_) {}
      sendRef.current = null;
    };
  }, [setMode, updateTelem, setConnected, driveRef, gHAngleRef, gVAngleRef, sendRef]);
}
