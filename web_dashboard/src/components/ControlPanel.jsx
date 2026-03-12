import { useTelemetry } from '../contexts/TelemetryContext';

function setSusp(sFrontTargetRef, sBackTargetRef, mode) {
  switch (mode) {
    case 'flat':
      sFrontTargetRef.current = 0;
      sBackTargetRef.current = 0;
      break;
    case 'low':
      sFrontTargetRef.current = 30;
      sBackTargetRef.current = 30;
      break;
    case 'high':
      sFrontTargetRef.current = 60;
      sBackTargetRef.current = 60;
      break;
    case 'rock':
      sFrontTargetRef.current = 60;
      sBackTargetRef.current = 30;
      break;
    default:
      break;
  }
}

export function ControlPanel() {
  const {
    suspLocked,
    setSuspLocked,
    frontCam,
    setFrontCam,
    backCam,
    setBackCam,
    gimbalCam,
    setGimbalCam,
    speedPct,
    setSpeedPct,
    sFrontTargetRef,
    sBackTargetRef,
  } = useTelemetry();

  const snap10 = (v) => Math.round((parseInt(v || 0, 10)) / 10) * 10;

  return (
    <div className="stack">
      <div className="ctl">
        <span>SUSPENSION LOCKED</span>
        <label className="switch">
          <input
            type="checkbox"
            checked={suspLocked}
            onChange={(e) => setSuspLocked(e.target.checked)}
          />
          <span className="knob"></span>
        </label>
      </div>
      <div className="ctl">
        <span>SUSPENSION POSITION</span>
        <div className="pill">
          <button type="button" onClick={() => setSusp(sFrontTargetRef, sBackTargetRef, 'flat')}>
            Flat
          </button>
          <button type="button" onClick={() => setSusp(sFrontTargetRef, sBackTargetRef, 'low')}>
            Low
          </button>
          <button type="button" onClick={() => setSusp(sFrontTargetRef, sBackTargetRef, 'high')}>
            High
          </button>
          <button type="button" onClick={() => setSusp(sFrontTargetRef, sBackTargetRef, 'rock')}>
            Rock
          </button>
        </div>
      </div>
      <div className="ctl">
        <span>FRONT CAMERA</span>
        <label className="switch">
          <input type="checkbox" checked={frontCam} onChange={(e) => setFrontCam(e.target.checked)} />
          <span className="knob"></span>
        </label>
      </div>
      <div className="ctl">
        <span>BACK CAMERA</span>
        <label className="switch">
          <input type="checkbox" checked={backCam} onChange={(e) => setBackCam(e.target.checked)} />
          <span className="knob"></span>
        </label>
      </div>
      <div className="ctl">
        <span>GIMBAL CAMERA</span>
        <label className="switch">
          <input type="checkbox" checked={gimbalCam} onChange={(e) => setGimbalCam(e.target.checked)} />
          <span className="knob"></span>
        </label>
      </div>
      <div className="ctl" style={{ flexDirection: 'column', alignItems: 'stretch' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <span>DRIVING SPEED</span>
          <span>{speedPct}%</span>
        </div>
        <input
          type="range"
          min="0"
          max="100"
          value={speedPct}
          step="10"
          className="slider"
          onChange={(e) => {
            const val = snap10(e.target.value);
            setSpeedPct(val);
          }}
        />
      </div>
    </div>
  );
}
