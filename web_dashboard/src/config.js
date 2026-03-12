const PI_HOST = typeof location !== 'undefined' ? (location.hostname || '10.42.0.191') : '10.42.0.191';
export const CAM_PORT = 8081;
export const WS_PORT = 8765;
export const WS_URL = `ws://${PI_HOST}:${WS_PORT}`;

export const CAMS = {
  cam0: {
    label: 'Front',
    url: `http://${PI_HOST}:${CAM_PORT}/stream?topic=/roo/front/image&type=ros_compressed`,
  },
  cam1: {
    label: 'Back',
    url: `http://${PI_HOST}:${CAM_PORT}/stream?topic=/roo/back/image&type=ros_compressed`,
  },
  cam2: {
    label: 'Gimbal',
    url: `http://${PI_HOST}:${CAM_PORT}/stream?topic=/roo/gimbal/image&type=ros_compressed`,
  },
};
