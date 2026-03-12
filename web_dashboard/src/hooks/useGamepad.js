import { useEffect } from 'react';
import { useTelemetry } from '../contexts/TelemetryContext';

export function useGamepad() {
  const { setSuspLocked, keyStateRef } = useTelemetry();

  useEffect(() => {
    const gamepadState = { index: null, lastButtons: [] };

    const onConnected = (e) => {
      if (gamepadState.index === null) gamepadState.index = e.gamepad.index;
    };

    const onDisconnected = (e) => {
      if (gamepadState.index === e.gamepad.index) gamepadState.index = null;
    };

    window.addEventListener('gamepadconnected', onConnected);
    window.addEventListener('gamepaddisconnected', onDisconnected);

    const checkTouchpad = () => {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      if (!pads) return;
      const gp = gamepadState.index != null && pads[gamepadState.index]
        ? pads[gamepadState.index]
        : pads.find((p) => p);
      if (!gp || !gp.buttons[17]) return;
      const touchPressed = !!gp.buttons[17]?.pressed;
      const lastTouch = gamepadState.lastButtons[17] || false;
      if (touchPressed && !lastTouch) {
        setSuspLocked((prev) => !prev);
      }
      gamepadState.lastButtons[17] = touchPressed;
    };

    const interval = setInterval(checkTouchpad, 100);

    return () => {
      window.removeEventListener('gamepadconnected', onConnected);
      window.removeEventListener('gamepaddisconnected', onDisconnected);
      clearInterval(interval);
    };
  }, [setSuspLocked]);
}
