import { TopBar } from './components/TopBar';
import { PowerCard } from './components/PowerCard';
import { ConnectionCard } from './components/ConnectionCard';
import { TiltCard } from './components/TiltCard';
import { EncodersCard } from './components/EncodersCard';
import { ControlPanel } from './components/ControlPanel';
import { CameraViewer } from './components/CameraViewer';
import { NirSection } from './components/NirSection';
import { useWebSocket } from './hooks/useWebSocket';
import { useTelemetryTick } from './hooks/useTelemetryTick';
import { useKeyboard } from './hooks/useKeyboard';
import { useGamepad } from './hooks/useGamepad';

function DashboardHooks() {
  useWebSocket();
  useTelemetryTick();
  useKeyboard();
  useGamepad();
  return null;
}

export function App() {
  return (
    <div className="app">
      <DashboardHooks />
      <TopBar />
      <section className="cards">
        <PowerCard />
        <ConnectionCard />
        <TiltCard />
        <EncodersCard />
      </section>
      <section className="lower">
        <ControlPanel />
        <CameraViewer />
      </section>
      <NirSection />
    </div>
  );
}
