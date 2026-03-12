import { useTelemetry } from '../contexts/TelemetryContext';

export function EncodersCard() {
  const { telem } = useTelemetry();

  const enc1 = telem.enc1_angle;
  const enc2 = telem.enc2_angle;

  return (
    <div className="card encoders">
      <div style={{ fontWeight: 700, opacity: 0.8, marginBottom: 6 }}>Gimbal Encoders</div>
      <div className="row" style={{ marginTop: 16 }}>
        <div style={{ flex: 1, textAlign: 'center' }}>
          <div style={{ opacity: 0.7 }}>Pitch (Enc 1)</div>
          <div className="big" style={{ color: 'var(--accent)' }}>
            {typeof enc1 === 'number' ? enc1.toFixed(1) : '0.0'}°
          </div>
        </div>
        <div style={{ flex: 1, textAlign: 'center' }}>
          <div style={{ opacity: 0.7 }}>Roll (Enc 2)</div>
          <div className="big" style={{ color: 'var(--accent)' }}>
            {typeof enc2 === 'number' ? enc2.toFixed(1) : '0.0'}°
          </div>
        </div>
      </div>
      <div style={{ textAlign: 'center', opacity: 0.5, fontSize: 11, marginTop: 20 }}>
        MT6701 Analog Feedback
      </div>
    </div>
  );
}
