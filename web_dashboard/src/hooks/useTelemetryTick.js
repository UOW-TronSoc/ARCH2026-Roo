import { useEffect, useRef } from 'react';
import { useTelemetry } from '../contexts/TelemetryContext';

function clamp(n, min, max) {
  return Math.min(max, Math.max(min, n));
}

function setKey(keyState, k, val) {
  const keys = ['w', 'W', 'a', 'A', 's', 'S', 'd', 'D', 'u', 'U', 'i', 'I', 'o', 'O', 'p', 'P'];
  const map = { w: 'w', W: 'w', a: 'a', A: 'a', s: 's', S: 's', d: 'd', D: 'd', u: 'u', U: 'u', i: 'i', I: 'i', o: 'o', O: 'o', p: 'p', P: 'p' };
  if (map[k] !== undefined) keyState[map[k]] = val;
  if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(k)) keyState[k] = val;
}

export function useTelemetryTick() {
  const {
    mode,
    updateTelem,
    speedPct,
    driveRef,
    gHAngleRef,
    gVAngleRef,
    sFrontTargetRef,
    sBackTargetRef,
    keyStateRef,
  } = useTelemetry();

  const tRef = useRef(0);
  const gamepadStateRef = useRef({ index: null, lastButtons: [] });

  useEffect(() => {
    function dz(v, dead = 0.12) {
      return Math.abs(v) < dead ? 0 : v;
    }

    function getGamepad() {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      if (!pads) return null;
      const idx = gamepadStateRef.current.index;
      if (idx != null && pads[idx]) return pads[idx];
      for (const p of pads) if (p) return p;
      return null;
    }

    const interval = setInterval(() => {
      const dt = 0.25;
      tRef.current += dt;
      const t = tRef.current;

      if (mode === 'SIM') {
        updateTelem({
          voltage_v: 12.1 + 0.3 * Math.sin(t / 2),
          current_a: 4.0 + 0.5 * Math.cos(t / 3),
          energy_mwh: 1500 + t * 10,
          pitch_deg: 10 * Math.sin(t / 2),
          roll_deg: 20 * Math.cos(t / 2),
          ping_ms: 140 + Math.floor(40 * Math.abs(Math.sin(t / 1.7))),
        });
      }

      const keyState = keyStateRef.current;
      let forward = (keyState.w ? 1 : 0) + (keyState.s ? -1 : 0);
      let turn = (keyState.d ? 1 : 0) + (keyState.a ? -1 : 0);

      const gp = getGamepad();
      if (gp) {
        const LX = dz(gp.axes[0] || 0);
        const LY = dz(gp.axes[1] || 0);
        forward = clamp(-LY, -1, 1);
        turn = clamp(LX, -1, 1);
      }

      const sp = (speedPct || 0) / 100;
      driveRef.current = {
        left: clamp(forward - turn, -1, 1) * sp,
        right: clamp(forward + turn, -1, 1) * sp,
      };

      if (gp) {
        const RX = dz(gp.axes[2] || 0);
        const RY = dz(gp.axes[3] || 0);
        const gRate = 20;
        gHAngleRef.current = clamp(gHAngleRef.current + RX * gRate * dt, -90, 90);
        gVAngleRef.current = clamp(gVAngleRef.current + (-RY) * gRate * dt, -90, 90);

        const b = gp.buttons;
        const L1 = b[4]?.pressed;
        const R1 = b[5]?.pressed;
        const L2v = typeof b[6]?.value === 'number' ? b[6].value : (b[6]?.pressed ? 1 : 0);
        const R2v = typeof b[7]?.value === 'number' ? b[7].value : (b[7]?.pressed ? 1 : 0);
        const trigDead = 0.15;
        const sRate = 30;
        const step = sRate * dt;

        if (L1) sFrontTargetRef.current = clamp(sFrontTargetRef.current - step, 0, 60);
        if (R1) sBackTargetRef.current = clamp(sBackTargetRef.current - step, 0, 60);
        if (L2v > trigDead) sFrontTargetRef.current = clamp(sFrontTargetRef.current + step * L2v, 0, 60);
        if (R2v > trigDead) sBackTargetRef.current = clamp(sBackTargetRef.current + step * R2v, 0, 60);

        gamepadStateRef.current.lastButtons[17] = !!b[17]?.pressed;
      }

      const gStep = 5;
      if (keyState.ArrowLeft) gHAngleRef.current = clamp(gHAngleRef.current - gStep, -90, 90);
      if (keyState.ArrowRight) gHAngleRef.current = clamp(gHAngleRef.current + gStep, -90, 90);
      if (keyState.ArrowUp) gVAngleRef.current = clamp(gVAngleRef.current + gStep, -90, 90);
      if (keyState.ArrowDown) gVAngleRef.current = clamp(gVAngleRef.current - gStep, -90, 90);

      const sStep = 0.625;
      if (keyState.u) sFrontTargetRef.current = clamp(sFrontTargetRef.current - sStep, 0, 60);
      if (keyState.i) sFrontTargetRef.current = clamp(sFrontTargetRef.current + sStep, 0, 60);
      if (keyState.o) sBackTargetRef.current = clamp(sBackTargetRef.current + sStep, 0, 60);
      if (keyState.p) sBackTargetRef.current = clamp(sBackTargetRef.current - sStep, 0, 60);
    }, 250);

    return () => clearInterval(interval);
  }, [mode, speedPct, updateTelem, driveRef, gHAngleRef, gVAngleRef, sFrontTargetRef, sBackTargetRef, keyStateRef]);

  return { gamepadStateRef };
}
