import { useTelemetry } from '../contexts/TelemetryContext';

export function ConnectionCard() {
  const { connected, telem } = useTelemetry();

  const ping = telem.ping_ms;

  return (
    <div className="card conn">
      <div style={{ fontWeight: 700, opacity: 0.8, marginBottom: 6 }}>Connection</div>
      <div className="circles">
        <div className="circle">
          <div>
            <div
              className={connected ? 'status-ok' : ''}
              style={{
                textAlign: 'center',
                fontSize: 20,
                color: connected ? undefined : '#ef4444',
                fontWeight: connected ? undefined : 700,
              }}
            >
              {connected ? 'Online' : 'Offline'}
            </div>
            <div style={{ textAlign: 'center', opacity: 0.7 }}>Status</div>
          </div>
        </div>
        <div className="circle">
          <div>
            <div className="big">
              {typeof ping === 'number' && ping > 0 ? Math.round(ping) : '---'}
            </div>
            <div style={{ textAlign: 'center', opacity: 0.7 }}>Ping (ms)</div>
          </div>
        </div>
      </div>
    </div>
  );
}
