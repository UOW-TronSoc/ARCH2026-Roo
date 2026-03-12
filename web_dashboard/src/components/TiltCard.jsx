import { useTelemetry } from '../contexts/TelemetryContext';

export function TiltCard() {
  const { telem } = useTelemetry();

  const pitch = telem.pitch_deg;
  const roll = telem.roll_deg;

  return (
    <div className="card tilt">
      <div style={{ fontWeight: 700, opacity: 0.8, marginBottom: 6 }}>Tilt</div>
      <div className="metric" style={{ flexDirection: 'column', alignItems: 'center', gap: 8 }}>
        <div
          className="tilt-line"
          style={typeof pitch === 'number' ? { transform: `rotate(${pitch.toFixed(1)}deg)` } : {}}
        />
        <div className="tilt-labels">
          <span>F</span>
          <span>B</span>
        </div>
      </div>
      <div className="metric" style={{ flexDirection: 'column', alignItems: 'center', gap: 8 }}>
        <div
          className="tilt-line"
          style={typeof roll === 'number' ? { transform: `rotate(${roll.toFixed(1)}deg)` } : {}}
        />
        <div className="tilt-labels">
          <span>L</span>
          <span>R</span>
        </div>
      </div>
      <div className="row" style={{ marginTop: 10 }}>
        <div style={{ flex: 1, textAlign: 'center' }}>
          <div style={{ opacity: 0.7 }}>Pitch</div>
          <div className="big">{typeof pitch === 'number' ? Math.round(pitch) : 0}</div>
          <div style={{ opacity: 0.7 }}>Degrees</div>
        </div>
        <div style={{ flex: 1, textAlign: 'center' }}>
          <div style={{ opacity: 0.7 }}>Roll</div>
          <div className="big">{typeof roll === 'number' ? Math.round(roll) : 0}</div>
          <div style={{ opacity: 0.7 }}>Degrees</div>
        </div>
      </div>
    </div>
  );
}
