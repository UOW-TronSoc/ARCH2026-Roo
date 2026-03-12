import { useEffect } from 'react';
import { useTelemetry } from '../contexts/TelemetryContext';

function setKey(keyState, k, val) {
  const map = { w: 'w', W: 'w', a: 'a', A: 'a', s: 's', S: 's', d: 'd', D: 'd', u: 'u', U: 'u', i: 'i', I: 'i', o: 'o', O: 'o', p: 'p', P: 'p' };
  if (map[k] !== undefined) keyState[map[k]] = val;
  if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(k)) keyState[k] = val;
}

export function useKeyboard() {
  const { keyStateRef } = useTelemetry();

  useEffect(() => {
    const keyState = keyStateRef.current;
    const keysToPrevent = ['w', 'W', 'a', 'A', 's', 'S', 'd', 'D', 'u', 'U', 'i', 'I', 'o', 'O', 'p', 'P', 'ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', ' '];

    const onKeydown = (e) => {
      const k = e.key;
      if (keysToPrevent.includes(k)) e.preventDefault();
      setKey(keyState, k, true);
    };

    const onKeyup = (e) => {
      setKey(keyState, e.key, false);
    };

    window.addEventListener('keydown', onKeydown, { passive: false });
    window.addEventListener('keyup', onKeyup);

    return () => {
      window.removeEventListener('keydown', onKeydown);
      window.removeEventListener('keyup', onKeyup);
    };
  }, [keyStateRef]);
}
