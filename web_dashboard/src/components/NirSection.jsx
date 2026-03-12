import { useState, useRef, useEffect } from 'react';

const BANDS_NM = [410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940];
const HIST_MAX = 180;
const COOLDOWN_MS = 10000;
const ACQ_MS = 8000;

function clamp(n, min, max) {
  return Math.min(max, Math.max(min, n));
}

function gauss(x, mu, sigma) {
  const z = (x - mu) / sigma;
  return Math.exp(-0.5 * z * z);
}

function randn() {
  let s = 0;
  for (let i = 0; i < 6; i++) s += Math.random() * 2 - 1;
  return s / 6;
}

function idxOf(nm) {
  let best = 0;
  let bd = 1e9;
  for (let i = 0; i < BANDS_NM.length; i++) {
    const d = Math.abs(BANDS_NM[i] - nm);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best;
}

const i680 = idxOf(680);
const i860 = idxOf(860);
const i900 = idxOf(900);
const i940 = idxOf(940);

function simSpectrum(tt) {
  const drift = 0.5 + 0.35 * Math.sin(tt / 9) + 0.15 * Math.sin(tt / 3.8);
  const ilmLevel = clamp(drift, 0.1, 0.95);

  const base = BANDS_NM.map((nm) => {
    const visBright = 0.55 + 0.1 * Math.sin(tt / 3);
    const nirRise = 0.12 * ((nm - 600) / 400);
    const undulate = 0.03 * Math.sin(tt + nm / 90);
    let y = visBright + nirRise + undulate;
    y *= 0.92 + 0.06 * Math.sin(tt / 2.2);
    return y;
  });

  const spec = BANDS_NM.map((nm, idx) => {
    const dark = 0.22 * ilmLevel;
    const visWeight = clamp((700 - nm) / 400, 0, 1);
    const visDark = dark * (0.9 * visWeight + 0.35 * (1 - visWeight));
    const nirHump = 0.1 * ilmLevel * gauss(nm, 900, 85);
    const dip760 = 0.05 * ilmLevel * gauss(nm, 760, 35);

    let y = base[idx] - visDark + nirHump - dip760;
    const n = 0.015 * randn();
    y = clamp(y + n, 0.02, 1.2);
    return y;
  });

  return spec;
}

function computeFeatures(spec) {
  const v680 = spec[i680];
  const v860 = spec[i860];
  const v900 = spec[i900];
  const v940 = spec[i940];

  const slope = (v860 - v680) / (860 - 680);
  const ratio = v680 > 1e-6 ? v860 / v680 : 0;
  const broad = (Math.abs(v940 - v900) + Math.abs(v900 - v860)) / 2;

  const darkVis = clamp((0.75 - v680) / 0.75, 0, 1);
  const nirCurv = clamp(broad / 0.18, 0, 1);
  const slopeN = clamp((slope + 0.0001) / 0.0012, 0, 1);

  const score = clamp(0.55 * darkVis + 0.3 * nirCurv + 0.15 * (1 - slopeN), 0, 1);
  return { slope, ratio, broad, score };
}

function fmtTime(d) {
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
}

export function NirSection() {
  const [hist, setHist] = useState([]);
  const [spec, setSpec] = useState(BANDS_NM.map(() => 0));
  const [features, setFeatures] = useState({ slope: 0, ratio: 0, broad: 0, score: 0 });
  const [status, setStatus] = useState('IDLE');
  const [measuring, setMeasuring] = useState(false);
  const ttRef = useRef(0);
  const lastReadAtRef = useRef(0);
  const canvasRef = useRef(null);

  const maxV = Math.max(...spec, 1e-6);
  const normBars = spec.map((v) => clamp(v / maxV, 0.05, 1));

  const handleTakeReading = () => {
    const now = Date.now();
    if (now - lastReadAtRef.current < COOLDOWN_MS) {
      setStatus('COOLDOWN');
      return;
    }
    lastReadAtRef.current = now;
    setMeasuring(true);
    setStatus('MEASURING');
  };

  useEffect(() => {
    if (!measuring) return;
    const timer = setTimeout(() => {
      ttRef.current += 0.12;
      const newSpec = simSpectrum(ttRef.current);
      const f = computeFeatures(newSpec);
      const pct = Math.round(f.score * 100);
      setSpec(newSpec);
      setFeatures(f);
      setHist((prev) => {
        const next = [
          ...prev,
          { t: new Date(), pct, ratio: f.ratio, slope: f.slope, broad: f.broad },
        ];
        if (next.length > HIST_MAX) next.shift();
        return next;
      });
      setStatus('RECEIVED');
      setMeasuring(false);
    }, ACQ_MS);
    return () => clearTimeout(timer);
  }, [measuring]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(300, Math.floor(rect.width * dpr));
    const h = Math.max(180, Math.floor(rect.height * dpr));

    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;

    ctx.clearRect(0, 0, w, h);

    const padL = 48;
    const padR = 16;
    const padT = 14;
    const padB = 28;
    const pw = w - padL - padR;
    const ph = h - padT - padB;

    [0, 25, 50, 75, 100].forEach((v) => {
      const y = padT + (1 - v / 100) * ph;
      ctx.strokeStyle = 'rgba(255,255,255,0.07)';
      ctx.lineWidth = 1 * dpr;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + pw, y);
      ctx.stroke();

      ctx.fillStyle = 'rgba(229,231,235,0.55)';
      ctx.font = `${Math.round(11 * dpr)}px system-ui`;
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(v), padL - 8 * dpr, y);
    });

    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    ctx.moveTo(padL, padT);
    ctx.lineTo(padL, padT + ph);
    ctx.lineTo(padL + pw, padT + ph);
    ctx.stroke();

    if (hist.length < 2) {
      ctx.fillStyle = 'rgba(229,231,235,0.55)';
      ctx.font = `${Math.round(12 * dpr)}px system-ui`;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText('Waiting for history…', padL + 8 * dpr, padT + 8 * dpr);
      return;
    }

    const n = hist.length;
    const xFor = (i) => padL + (i / (n - 1)) * pw;
    const yForPct = (pct) => padT + (1 - pct / 100) * ph;

    ctx.strokeStyle = 'rgba(43,102,246,0.95)';
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = xFor(i);
      const y = yForPct(hist[i].pct);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = 'rgba(43,102,246,1)';
    ctx.beginPath();
    ctx.arc(xFor(n - 1), yForPct(hist[n - 1].pct), 3.5 * dpr, 0, Math.PI * 2);
    ctx.fill();
  }, [hist]);

  const histRows = hist
    .slice()
    .reverse()
    .slice(0, 12);

  const pct = Math.round(features.score * 100);

  return (
    <section className="nir">
      <div className="nir-head">
        <div>
          <div className="nir-title">NIR Spectroscopy (SparkFun Triad)</div>
          <div className="nir-sub">
            Simulated AS7265x 18-channel spectrum + ilmenite-focused features
          </div>
        </div>
        <div style={{ display: 'flex', gap: 10, alignItems: 'center', flexWrap: 'wrap' }}>
          <button
            type="button"
            className="nir-btn"
            onClick={handleTakeReading}
            disabled={measuring}
          >
            {measuring ? 'Measuring…' : 'Take Reading'}
          </button>
          <span className="status-badge">{status}</span>
        </div>
      </div>

      <div className="nir-grid">
        <div className="nir-panel">
          <div
            style={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'baseline',
              gap: 10,
              flexWrap: 'wrap',
            }}
          >
            <div style={{ fontWeight: 800, letterSpacing: 0.3 }}>Spectrum (18 bands)</div>
            <div className="status-badge">NIR Sensor</div>
          </div>

          <div className="nir-spark" aria-label="Spectrum bars">
            {normBars.map((n, idx) => (
              <div
                key={idx}
                className="b"
                style={{ height: `${n * 100}%` }}
              />
            ))}
          </div>

          <div className="nir-metric" style={{ marginTop: 12 }}>
            <div className="k">Ilmenite Likelihood History</div>
            <canvas
              ref={canvasRef}
              className="nir-canvas"
              width={900}
              height={220}
              style={{ marginTop: 10 }}
            />
          </div>

          <table className="nir-table" aria-label="Ilmenite history table" style={{ marginTop: 12 }}>
            <thead>
              <tr>
                <th style={{ textAlign: 'left' }}>Time</th>
                <th>Ilm %</th>
                <th>860/680</th>
                <th>Slope</th>
                <th>Broad</th>
              </tr>
            </thead>
            <tbody>
              {histRows.map((s, idx) => (
                <tr key={idx}>
                  <td style={{ textAlign: 'left' }}>{fmtTime(s.t)}</td>
                  <td>{s.pct}%</td>
                  <td>{s.ratio.toFixed(3)}</td>
                  <td>{s.slope.toFixed(5)}</td>
                  <td>{s.broad.toFixed(3)}</td>
                </tr>
              ))}
            </tbody>
          </table>

          <table className="nir-table" aria-label="AS7265x band values">
            <thead>
              <tr>
                <th>Band (nm)</th>
                <th>Value</th>
                <th>Norm</th>
              </tr>
            </thead>
            <tbody>
              {BANDS_NM.map((nm, idx) => (
                <tr key={idx}>
                  <td>{nm}</td>
                  <td>{spec[idx]?.toFixed(3) ?? '0.000'}</td>
                  <td>{((spec[idx] ?? 0) / maxV).toFixed(3)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <div className="nir-panel">
          <div style={{ fontWeight: 800, letterSpacing: 0.3 }}>Ilmenite Indicators</div>

          <div style={{ marginTop: 10 }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <div style={{ color: 'var(--ink-dim)', fontSize: 12, fontWeight: 700 }}>
                Ilmenite Likelihood (heuristic)
              </div>
              <div style={{ fontWeight: 800 }}>{pct}%</div>
            </div>
            <div className="nir-meter" style={{ marginTop: 8 }}>
              <div style={{ width: `${pct}%` }}></div>
            </div>
          </div>

          <div className="nir-metrics">
            <div className="nir-metric">
              <div className="k">Red/NIR slope</div>
              <div className="v">{features.slope.toFixed(5)}</div>
              <div className="nir-sub">~(860–680) / Δnm</div>
            </div>
            <div className="nir-metric">
              <div className="k">Band ratio</div>
              <div className="v">{features.ratio.toFixed(3)}</div>
              <div className="nir-sub">860 / 680</div>
            </div>
            <div className="nir-metric">
              <div className="k">Broadness</div>
              <div className="v">{features.broad.toFixed(3)}</div>
              <div className="nir-sub">NIR spread (860–940)</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
