import { useTelemetry } from '../contexts/TelemetryContext';

export function TopBar() {
  const { mode, connected } = useTelemetry();

  return (
    <div className="topbar">
      <div className="mode">MODE: {mode}</div>
      <div className="status-badge">
        WebBridge: {connected ? 'Connected' : 'Disconnected'}
      </div>
    </div>
  );
}
