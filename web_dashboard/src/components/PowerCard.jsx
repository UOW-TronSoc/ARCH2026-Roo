import { useTelemetry } from '../contexts/TelemetryContext';

export function PowerCard() {
  const { telem, resetTaskPower } = useTelemetry();

  const v = telem.voltage_v;
  const i = telem.current_a;
  const e = telem.energy_mwh;

  return (
    <div className="card power">
      <div style={{ fontWeight: 700, opacity: 0.8 }}>Power</div>
      <div className="metric">
        <span>Voltage</span>
        <span className="big">{typeof v === 'number' ? v.toFixed(2) : '0.00'}</span>
      </div>
      <div className="metric" style={{ marginTop: -8, opacity: 0.7, fontSize: 12 }}>
        <span>Volts</span>
        <span></span>
      </div>
      <div className="metric" style={{ marginTop: 2 }}>
        <span>Current</span>
        <span className="big">{typeof i === 'number' ? i.toFixed(2) : '0.00'}</span>
      </div>
      <div className="metric" style={{ marginTop: -8, opacity: 0.7, fontSize: 12 }}>
        <span>Amps</span>
        <span></span>
      </div>
      <div className="metric" style={{ marginTop: 2 }}>
        <span>Energy</span>
        <span className="medium">{typeof e === 'number' ? e.toFixed(2) : '0.00'}</span>
      </div>
      <button className="reset-btn" onClick={resetTaskPower}>
        RESET TASK ENERGY
      </button>
    </div>
  );
}
