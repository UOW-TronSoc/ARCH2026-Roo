import React from 'react';
import ReactDOM from 'react-dom/client';
import { TelemetryProvider } from './contexts/TelemetryContext';
import { App } from './App';
import './index.css';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <TelemetryProvider>
      <App />
    </TelemetryProvider>
  </React.StrictMode>
);
