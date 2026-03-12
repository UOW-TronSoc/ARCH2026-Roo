import { useState } from 'react';
import { CAMS } from '../config';
import { useTelemetry } from '../contexts/TelemetryContext';

export function CameraViewer() {
  const { frontCam, setFrontCam, backCam, setBackCam, gimbalCam, setGimbalCam } = useTelemetry();
  const [camMode, setCamMode] = useState('TG');

  const activeIds = [];
  if (frontCam) activeIds.push('cam0');
  if (backCam) activeIds.push('cam1');
  if (gimbalCam) activeIds.push('cam2');

  const viewerClass = camMode === 'ST' ? 'viewer stage' : 'viewer';

  return (
    <div className="cam-wrap">
      <div className="cam-bar">
        <label className="toggle">
          <input type="checkbox" checked={frontCam} onChange={(e) => setFrontCam(e.target.checked)} />
          Front
        </label>
        <label className="toggle">
          <input type="checkbox" checked={backCam} onChange={(e) => setBackCam(e.target.checked)} />
          Back
        </label>
        <label className="toggle">
          <input type="checkbox" checked={gimbalCam} onChange={(e) => setGimbalCam(e.target.checked)} />
          Gimbal
        </label>
        <div className="spacer"></div>
        <div className="mode-box">
          <span style={{ opacity: 0.7, fontSize: 12 }}>TG</span>
          <label className="switch sm">
            <input
              type="checkbox"
              checked={camMode === 'ST'}
              onChange={(e) => setCamMode(e.target.checked ? 'ST' : 'TG')}
            />
            <span className="knob"></span>
          </label>
          <span style={{ opacity: 0.7, fontSize: 12 }}>ST</span>
        </div>
        <ClearButton />
      </div>
      <div className={viewerClass} style={{ gridTemplateColumns: activeIds.length <= 1 ? '1fr' : '1fr 1fr' }}>
        {activeIds.length === 0 ? (
          <div className="placeholder">No cameras selected</div>
        ) : camMode === 'ST' ? (
          <StageLayout />
        ) : (
          activeIds.map((id) => {
            const { label, url } = CAMS[id];
            return (
              <div key={id} className="tile">
                <img alt={label} src={url} decoding="async" loading="eager" referrerPolicy="no-referrer" />
                <div className="badge">{label}</div>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}

function ClearButton() {
  const { setFrontCam, setBackCam, setGimbalCam } = useTelemetry();
  return (
    <button
      type="button"
      className="btn"
      onClick={(e) => {
        e.preventDefault();
        e.stopPropagation();
        setFrontCam(false);
        setBackCam(false);
        setGimbalCam(false);
      }}
    >
      Clear
    </button>
  );
}

function StageLayout() {
  const ids = ['cam0', 'cam1', 'cam2'];
  return (
    <>
      <div>
        {ids.slice(0, 1).map((id) => {
          const { label, url } = CAMS[id];
          return (
            <div key={id} className="tile">
              <img alt={label} src={url} />
              <div className="badge">{label}</div>
            </div>
          );
        })}
      </div>
      <div style={{ display: 'grid', gap: 12 }}>
        {ids.slice(1).map((id) => {
          const { label, url } = CAMS[id];
          return (
            <div key={id} className="tile">
              <img alt={label} src={url} />
              <div className="badge">{label}</div>
            </div>
          );
        })}
      </div>
    </>
  );
}
