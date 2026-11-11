/* =========================================================================
 * web/js/app.js
 * Controlador Maestro del Observatorio Celeste 3D:
 *  - Autoconexión WiFi cada 10s hacia ESP32-S3 (HTTP GET /api/gps, SSE, WS)
 *  - Normalizador universal de JSON (soporta formato plano del ESP32-S3 y anidado)
 *  - Interpolación de 1s del Reloj Atómico GPS entre paquetes de 10s
 *  - Modo Demo de respaldo con la Trama Real Certificada del u-blox NEO-6M-0-001
 * ========================================================================= */
(function (global) {
  'use strict';

  /**
   * Trama de respaldo para demostración del u-blox NEO-6M-0-001
   * (Estación de Referencia GNSS / Coordenadas de prueba WGS-84)
   */
  const DEFAULT_TELEMETRY = {
    seq: 0,
    hostname: 'esp32-gps.local',
    ip: '192.168.4.1',
    trusted: true,
    certifiedEpochs: 703,
    streak: 250,
    lat: 0.0000000,
    lon: 0.0000000,
    hMSL: 0.000,
    height: 0.000,
    geoidSep: 0.000,
    ecefX: 6378137.00,
    ecefY: 0.00,
    ecefZ: 0.00,
    hAcc: 12.928,
    vAcc: 14.850,
    pAcc: 19.690,
    stdLat: 7.200,
    stdLon: 11.000,
    stdAlt: 15.000,
    rangeRms: 46.000,
    gSpeed: 0.170,
    speed3D: 0.180,
    sAcc: 0.760,
    velN: -0.120,
    velE: 0.120,
    velD: -0.050,
    climbRate: 0.050,
    ecefVX: 0.140,
    ecefVY: -0.030,
    ecefVZ: -0.120,
    heading: 0.40,
    cAcc: 99.49,
    headingStatic: false,
    fixType: 3,
    fixName: 'Fix 3D',
    ttff: 94849,
    numSVUsed: 6,
    numSVView: 15,
    numSVEph: 9,
    gdop: 2.40,
    pdop: 2.15,
    tdop: 1.06,
    vdop: 1.76,
    hdop: 1.24,
    ndop: 0.77,
    edop: 0.97,
    raimErrLat: 7.20,
    raimErrLon: 10.70,
    raimErrAlt: 14.80,
    raimFailedSV: 0,
    utcIso: '2026-11-09T15:57:03Z',
    localIso: '2026-11-09T10:57:03',
    gpsWeek: 2444,
    iTOW: 143841000,
    fTOW: -158190,
    leapS: 18,
    tAccNs: 27,
    clkBiasNs: -841810,
    clkDriftPpb: -5603,
    fAccPsS: 1196,
    tpGranNs: 21,
    qErrPs: 4513,
    healthMask: '0xFFFFFFFF',
    klobValid: true,
    klobAlpha: [0.0, 0.0, 0.0, 0.0],
    klobBeta: [0.0, 0.0, 0.0, 0.0],
    rxBytes: 192343,
    txBytes: 1230,
    uartErrors: 0,
    antStatus: 'OK',
    noisePerMS: 104,
    agcPct: 27.1,
    jamPct: 3.9,
    satellites: [
      { chn: 0,  prn: 1,   type: 'GPS',  used: true,  qid: 7, cno: 31, elev: 44,  azim: 344, ageEph: 2,  prResCm: 50,   grsM: 0.5  },
      { chn: 1,  prn: 2,   type: 'GPS',  used: false, qid: 1, cno: 0,  elev: 26,  azim: 25,  ageEph: 15, prResCm: 0,    grsM: null },
      { chn: 2,  prn: 3,   type: 'GPS',  used: true,  qid: 7, cno: 25, elev: 42,  azim: 325, ageEph: 2,  prResCm: 29,   grsM: 0.3  },
      { chn: 3,  prn: 4,   type: 'GPS',  used: false, qid: 6, cno: 14, elev: 76,  azim: 109, ageEph: 3,  prResCm: 0,    grsM: null },
      { chn: 4,  prn: 6,   type: 'GPS',  used: false, qid: 1, cno: 0,  elev: 2,   azim: 193, ageEph: 15, prResCm: 0,    grsM: null },
      { chn: 5,  prn: 8,   type: 'GPS',  used: false, qid: 1, cno: 0,  elev: 13,  azim: 68,  ageEph: 15, prResCm: 0,    grsM: null },
      { chn: 6,  prn: 9,   type: 'GPS',  used: true,  qid: 7, cno: 15, elev: 69,  azim: 192, ageEph: 2,  prResCm: 215,  grsM: 2.1  },
      { chn: 7,  prn: 14,  type: 'GPS',  used: false, qid: 1, cno: 0,  elev: 16,  azim: 317, ageEph: 15, prResCm: 0,    grsM: null },
      { chn: 8,  prn: 16,  type: 'GPS',  used: false, qid: 1, cno: 0,  elev: 28,  azim: 142, ageEph: 15, prResCm: 0,    grsM: null },
      { chn: 9,  prn: 17,  type: 'GPS',  used: true,  qid: 7, cno: 32, elev: 40,  azim: 229, ageEph: 2,  prResCm: -116, grsM: -1.1 },
      { chn: 10, prn: 19,  type: 'GPS',  used: true,  qid: 7, cno: 30, elev: 31,  azim: 205, ageEph: 2,  prResCm: 24,   grsM: 0.2  },
      { chn: 12, prn: 21,  type: 'GPS',  used: true,  qid: 7, cno: 25, elev: 65,  azim: 332, ageEph: 2,  prResCm: -135, grsM: -1.3 },
      { chn: 15, prn: 31,  type: 'GPS',  used: false, qid: 6, cno: 12, elev: 6,   azim: 132, ageEph: 3,  prResCm: 0,    grsM: null },
      { chn: 16, prn: 131, type: 'SBAS', used: false, qid: 6, cno: 24, elev: -91, azim: 0,   ageEph: 4,  prResCm: 0,    grsM: null },
      { chn: 17, prn: 138, type: 'SBAS', used: false, qid: 6, cno: 26, elev: 48,  azim: 292, ageEph: 4,  prResCm: 0,    grsM: null }
    ]
  };

  /**
   * Normaliza cualquier JSON entrante (tanto el JSON plano emitido por el ESP32-S3
   * en /api/gps como cualquier objeto anidado) a una estructura uniforme con
   * propiedades planas y sub-objetos [A]..[G].
   */
  function normalizeTelemetry(raw) {
    if (!raw || typeof raw !== 'object') return normalizeTelemetry(DEFAULT_TELEMETRY);

    const p = raw.position   || raw;
    const k = raw.kinematics || raw;
    const g = raw.geometry   || raw;
    const t = raw.time       || raw;
    const i = raw.ionosphere || raw;
    const h = raw.hardware   || raw;

    const rawSats = Array.isArray(raw.satellites) ? raw.satellites : [];
    const satellites = rawSats.map((s) => {
      const qi = s.qid !== undefined ? s.qid : (s.qi !== undefined ? s.qi : 0);
      const grsResM = s.grsM !== undefined ? s.grsM : (s.grsResM !== undefined ? s.grsResM : null);
      const hasEph = (s.ageEph !== undefined && s.ageEph < 15) || s.orb === 'EPH' || qi >= 4;
      return {
        chn: s.chn ?? 0,
        prn: s.prn ?? 0,
        type: s.type || (s.prn >= 120 ? 'SBAS' : 'GPS'),
        used: Boolean(s.used),
        healthy: s.healthy !== undefined ? Boolean(s.healthy) : true,
        dgps: s.dgps !== undefined ? Boolean(s.dgps) : Boolean((s.lck || 0) > 0),
        qi: qi,
        qid: qi,
        lck: s.lck ?? 0,
        cno: s.cno ?? 0,
        elev: s.elev ?? 0,
        azim: s.azim ?? 0,
        orb: s.orb || (hasEph ? 'EPH' : 'ALM'),
        prResCm: s.prResCm ?? 0,
        grsResM: grsResM
      };
    });

    const position = {
      lat: Number(p.lat ?? 0.0),
      lon: Number(p.lon ?? 0.0),
      hMSL: Number(p.hMSL ?? 0.0),
      height: Number(p.height ?? 0.0),
      geoidSep: Number(p.geoidSep ?? 0.0),
      ecefX: Number(p.ecefX ?? 6378137.0),
      ecefY: Number(p.ecefY ?? 0.0),
      ecefZ: Number(p.ecefZ ?? 0.0),
      hAcc: Number(p.hAcc ?? 12.928),
      vAcc: Number(p.vAcc ?? 14.850),
      pAcc: Number(p.pAcc ?? 19.690),
      stdLat: Number(p.stdLat ?? 7.20),
      stdLon: Number(p.stdLon ?? 11.00),
      stdAlt: Number(p.stdAlt ?? 15.00),
      rangeRms: Number(p.rangeRms ?? 46.00),
      navStat: p.navStat || ((g.fixType ?? 3) >= 3 ? 'G3' : 'G2'),
      gllMode: p.gllMode || 'A'
    };

    const kinematics = {
      gSpeed: Number(k.gSpeed ?? 0),
      speed3D: Number(k.speed3D ?? 0),
      sAcc: Number(k.sAcc ?? 0),
      velN: Number(k.velN ?? 0),
      velE: Number(k.velE ?? 0),
      velD: Number(k.velD ?? 0),
      climbRate: Number(k.climbRate ?? -(k.velD ?? 0)),
      ecefVX: Number(k.ecefVX ?? 0),
      ecefVY: Number(k.ecefVY ?? 0),
      ecefVZ: Number(k.ecefVZ ?? 0),
      heading: Number(k.heading ?? 0),
      cAcc: Number(k.cAcc ?? 0),
      stationary: Boolean(k.headingStatic ?? k.stationary ?? false)
    };

    const geometry = {
      fixType: Number(g.fixType ?? 3),
      fixName: g.fixName || 'Fix 3D',
      ttff: Number(g.ttff ?? 0),
      numSVUsed: Number(g.numSVUsed ?? satellites.filter((x) => x.used).length),
      numSVView: Number(g.numSVView ?? satellites.length),
      numSVEph: Number(g.numSVEph ?? 0),
      gdop: Number(g.gdop ?? 0),
      pdop: Number(g.pdop ?? 0),
      tdop: Number(g.tdop ?? 0),
      vdop: Number(g.vdop ?? 0),
      hdop: Number(g.hdop ?? 0),
      ndop: Number(g.ndop ?? 0),
      edop: Number(g.edop ?? 0),
      raimErrLat: Number(g.raimErrLat ?? position.stdLat),
      raimErrLon: Number(g.raimErrLon ?? position.stdLon),
      raimErrAlt: Number(g.raimErrAlt ?? position.stdAlt),
      raimFailedSV: Number(g.raimFailedSV ?? 0)
    };

    const time = {
      utcIso: t.utcIso || '2026-11-09T15:57:03Z',
      localIso: t.localIso || '2026-11-09T10:57:03',
      gpsWeek: Number(t.gpsWeek ?? 2444),
      iTOW: Number(t.iTOW ?? 0),
      fTOW: Number(t.fTOW ?? 0),
      leapS: Number(t.leapS ?? 18),
      tAccNs: Number(t.tAccNs ?? 27),
      clkBiasNs: Number(t.clkBiasNs ?? 0),
      clkDriftPpb: Number(t.clkDriftPpb ?? 0),
      fAccPsPerS: Number(t.fAccPsS ?? t.fAccPsPerS ?? 0),
      qErrPs: Number(t.qErrPs ?? 0),
      certifiedEpochs: Number(raw.certifiedEpochs ?? t.certifiedEpochs ?? 0),
      consecutiveValidEpochs: Number(raw.streak ?? t.consecutiveValidEpochs ?? 0)
    };

    const ionosphere = {
      valid: Boolean(i.klobValid ?? i.valid ?? true),
      healthMask: i.healthMask || '0xFFFFFFFF',
      klobAlpha: Array.isArray(i.klobAlpha) ? i.klobAlpha : [0, 0, 0, 0],
      klobBeta: Array.isArray(i.klobBeta) ? i.klobBeta : [0, 0, 0, 0]
    };

    const hardware = {
      antStatus: h.antStatus || 'OK',
      uartErrors: Number(h.uartErrors ?? 0),
      rxBytesTotal: Number(h.rxBytes ?? h.rxBytesTotal ?? 0),
      txBytesTotal: Number(h.txBytes ?? h.txBytesTotal ?? 0),
      noisePerMS: Number(h.noisePerMS ?? 0),
      agcPct: Number(h.agcPct ?? 0),
      jamPct: Number(h.jamPct ?? 0)
    };

    return {
      seq: raw.seq ?? 0,
      ip: raw.ip || '',
      hostname: raw.hostname || 'esp32-gps.local',
      intervalMs: Number(raw.intervalMs ?? 1000),
      epochUbxMs: Number(raw.epochUbxMs ?? 34.2),
      epochBurstMs: Number(raw.epochBurstMs ?? 98.4),
      epochFrames: Number(raw.epochFrames ?? 18),
      epochBytes: Number(raw.epochBytes ?? 1120),
      jsonBuildUs: Number(raw.jsonBuildUs ?? 480),
      phaseName: raw.phaseName || '',
      trusted: raw.trusted !== undefined ? Boolean(raw.trusted) : true,
      position,
      kinematics,
      geometry,
      time,
      ionosphere,
      hardware,
      satellites
    };
  }

  class ObservatorioApp {
    constructor() {
      this.telemetry = normalizeTelemetry(DEFAULT_TELEMETRY);
      this.isLiveConnected = false;
      this.isDemoMode = true;
      this.useManualTimeOverride = false;
      this.manualLocalMinutes = 10 * 60 + 57;
      this.selectedPrn = null;

      this.pollIntervalId = null;
      this.clockTickId = null;
      this.ws = null;
      this.sse = null;
      this.inFlightFetch = false;
      this.lastSeenITOW = -1;
      this.lastRxBytes = 0;
      this.lastHttpRttMs = 0;
      this.lastPacketTimestamp = performance.now();
      this.lastAnchorUtcMs = Date.parse(this.telemetry.time.utcIso);
      this.cyclePeriodMs = 1000;

      document.body.setAttribute('data-active-drawer', 'left');

      this._initScene();
      this._bindUI();
      this._applyTelemetry(this.telemetry);
      this._startCountdownAndClockLoop();

      // Autoconectar inmediatamente en Tiempo Real al ESP32-S3 al abrir la página
      const defaultHost = document.getElementById('esp-host-input').value.trim() || 'esp32-gps.local';
      this.connectToEsp32(defaultHost, 'realtime');
    }

    _initScene() {
      const container = document.getElementById('canvas-container');
      const labelsLayer = document.getElementById('labels-layer');
      const tooltip = document.getElementById('sat-tooltip');

      this.scene3d = new global.ObservatorioScene3D(container, labelsLayer, tooltip);
      this.scene3d.onSatelliteSelect = (prn) => {
        this.selectedPrn = prn;
        this._highlightTableRow(prn);
      };
    }

    _bindUI() {
      // Botones de modo de cámara
      const btnOrbit = document.getElementById('btn-cam-orbit');
      const btnFishUp = document.getElementById('btn-cam-fisheye-up');
      const btnFishTop = document.getElementById('btn-cam-fisheye-top');
      const allCamBtns = [btnOrbit, btnFishUp, btnFishTop];

      const setActiveCamBtn = (activeBtn, mode) => {
        allCamBtns.forEach((b) => b.classList.remove('active'));
        activeBtn.classList.add('active');
        this.scene3d.setCameraMode(mode);
      };

      btnOrbit.addEventListener('click', () => setActiveCamBtn(btnOrbit, 'orbit'));
      btnFishUp.addEventListener('click', () => setActiveCamBtn(btnFishUp, 'fisheye-up'));
      btnFishTop.addEventListener('click', () => setActiveCamBtn(btnFishTop, 'fisheye-top'));

      // Mostrar / Ocultar paneles HUD laterales
      const btnTogglePanels = document.getElementById('btn-toggle-panels');
      btnTogglePanels.addEventListener('click', () => {
        document.body.classList.toggle('panels-collapsed');
        const curDrawer = document.body.getAttribute('data-active-drawer');
        if (window.innerWidth <= 1180) {
          const next = curDrawer === 'none' ? 'left' : 'none';
          this._setActiveDrawerTab(next);
        }
      });

      // Selector de pestañas responsive para pantallas medianas/pequeñas
      document.querySelectorAll('.mobile-panel-switcher .btn-tab').forEach((btn) => {
        btn.addEventListener('click', () => {
          const target = btn.getAttribute('data-drawer') || 'left';
          document.body.classList.remove('panels-collapsed');
          this._setActiveDrawerTab(target);
        });
      });

      // Controles de conexión WiFi al ESP32-S3
      document.getElementById('btn-connect').addEventListener('click', () => {
        const host = document.getElementById('esp-host-input').value.trim();
        const proto = document.getElementById('esp-proto-select').value;
        this.connectToEsp32(host, proto);
      });

      document.getElementById('btn-demo-mode').addEventListener('click', () => {
        this.activateDemoMode();
      });

      // Control deslizante de hora solar / lunar
      const timeSlider = document.getElementById('time-slider');
      timeSlider.addEventListener('input', (ev) => {
        this.useManualTimeOverride = true;
        this.manualLocalMinutes = parseInt(ev.target.value, 10);
        this._updateAstronomyFromCurrentState();
      });

      document.querySelectorAll('.btn-preset').forEach((btn) => {
        btn.addEventListener('click', () => {
          const mins = parseInt(btn.getAttribute('data-minutes'), 10);
          this.useManualTimeOverride = true;
          this.manualLocalMinutes = mins;
          timeSlider.value = String(mins);
          this._updateAstronomyFromCurrentState();
        });
      });

      document.getElementById('btn-sync-real-time').addEventListener('click', () => {
        this.useManualTimeOverride = false;
        this._updateAstronomyFromCurrentState();
      });

      // Selector de proveedor de mapas 3D y escala del elipsoide
      document.getElementById('map-provider-select').addEventListener('change', (ev) => {
        const pos = this.telemetry.position || {};
        this.scene3d.updateGroundMap(pos.lat || 0.0, pos.lon || 0.0, ev.target.value);
      });

      document.getElementById('ellipsoid-scale-select').addEventListener('change', (ev) => {
        this.scene3d.setEllipsoidScale(parseFloat(ev.target.value));
      });
    }

    _setActiveDrawerTab(drawerName) {
      document.body.setAttribute('data-active-drawer', drawerName);
      document.querySelectorAll('.mobile-panel-switcher .btn-tab').forEach((b) => {
        b.classList.toggle('active', b.getAttribute('data-drawer') === drawerName);
      });
    }

    /**
     * Conecta por WiFi en Tiempo Real al ESP32-S3 (Push SSE /events + sondeo rápido /api/gps)
     */
    async connectToEsp32(rawHost, proto = 'realtime') {
      this._disconnectTransports();

      let cleanHost = (rawHost || '')
        .replace(/^https?:\/\//i, '')
        .replace(/^ws:\/\//i, '')
        .replace(/\/.*$/, '')
        .trim();
      if (!cleanHost) cleanHost = 'esp32-gps.local';

      const statusText = document.getElementById('conn-status-text');
      const orb = document.getElementById('conn-orb');
      const pill = document.getElementById('mode-pill');

      statusText.textContent = `Conectando en tiempo real a ${cleanHost}...`;

      const handleIncomingData = (rawJson, rttMs = 0) => {
        const norm = normalizeTelemetry(rawJson);
        if (rttMs > 0) this.lastHttpRttMs = rttMs;

        // Si el firmware aún no reporta epochBurstMs (>0), estimarlo físicamente de los bytes UART a 115200 bps
        if (!rawJson.epochBurstMs && this.lastRxBytes > 0 && norm.hardware.rxBytesTotal > this.lastRxBytes) {
          const deltaBytes = norm.hardware.rxBytesTotal - this.lastRxBytes;
          const secs = Math.max(1, (norm.intervalMs || 1000) / 1000);
          const bytesPerEpoch = Math.round(deltaBytes / secs);
          norm.epochBytes = bytesPerEpoch;
          norm.epochBurstMs = Number(((bytesPerEpoch * 10.0) / 115.2).toFixed(1));
          norm.epochUbxMs = Number((norm.epochBurstMs * 0.36).toFixed(1));
        }
        if (norm.hardware.rxBytesTotal > 0) {
          this.lastRxBytes = norm.hardware.rxBytesTotal;
        }

        this.cyclePeriodMs = Math.min(2000, Math.max(200, norm.intervalMs || 1000));
        this.isDemoMode = false;
        this.isLiveConnected = true;

        const isNewEpoch = norm.time.iTOW !== this.lastSeenITOW;
        if (isNewEpoch) {
          this.lastSeenITOW = norm.time.iTOW;
          this.lastPacketTimestamp = performance.now();
          const parsedUtc = Date.parse(norm.time.utcIso);
          if (!isNaN(parsedUtc)) {
            this.lastAnchorUtcMs = parsedUtc;
          }
          orb.className = 'status-orb online';
          pill.className = 'pill pill-live';
          pill.textContent = 'TIEMPO REAL (ESP32-S3)';

          const ipDisplay = norm.ip ? `${cleanHost} (${norm.ip})` : cleanHost;
          statusText.textContent = `En vivo: ${ipDisplay} · iTOW ${norm.time.iTOW} ms`;
          this._applyTelemetry(norm);
        }
      };

      const fetchSnapshotOnce = async (targetHost) => {
        if (this.inFlightFetch) return null;
        this.inFlightFetch = true;
        const t0 = performance.now();
        try {
          const resp = await fetch(`http://${targetHost}/api/gps`, {
            method: 'GET',
            mode: 'cors',
            cache: 'no-store'
          });
          if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
          const data = await resp.json();
          const rtt = performance.now() - t0;
          handleIncomingData(data, rtt);
          return data;
        } finally {
          this.inFlightFetch = false;
        }
      };

      let activeHost = cleanHost;
      try {
        const firstData = await fetchSnapshotOnce(activeHost);
        // Si obtuvimos la IP numérica directa, usarla para evitar demoras de resolución mDNS en Windows
        if (firstData && firstData.ip && firstData.ip !== '0.0.0.0' && activeHost.endsWith('.local')) {
          activeHost = firstData.ip;
        }
      } catch (firstErr) {
        this.isLiveConnected = false;
        orb.className = 'status-orb error';
        statusText.textContent = `Sin respuesta de ${cleanHost} (${firstErr.message}). Modo Demo activo.`;
        return;
        } else {
          this.isLiveConnected = false;
          orb.className = 'status-orb error';
          statusText.textContent = `Sin respuesta de ${cleanHost} (${firstErr.message}). Modo Demo activo.`;
          return;
        }
      }

      // Abrir stream Server-Sent Events (/events) para recibir cada trama en el instante exacto en que se cierra en el ESP32-S3
      if (proto === 'realtime' || proto === 'sse') {
        try {
          this.sse = new EventSource(`http://${activeHost}/events`);
          this.sse.onmessage = (ev) => {
            try { handleIncomingData(JSON.parse(ev.data), 2.5); } catch (_e) {}
          };
        } catch (_e) {}
      }

      // Sondeo rápido de baja latencia (cada 450 ms sobre la IP directa) para garantizar actualización inmediata por cada trama
      this.pollIntervalId = setInterval(() => {
        fetchSnapshotOnce(activeHost).catch((err) => {
          statusText.textContent = `Reintentando enlace con ${activeHost} (${err.message})...`;
        });
      }, 450);
    }

    activateDemoMode() {
      this._disconnectTransports();
      this.isDemoMode = true;
      this.isLiveConnected = false;
      this.cyclePeriodMs = 1000;
      this.lastPacketTimestamp = performance.now();

      const orb = document.getElementById('conn-orb');
      const pill = document.getElementById('mode-pill');
      const statusText = document.getElementById('conn-status-text');

      orb.className = 'status-orb';
      pill.className = 'pill pill-demo';
      pill.textContent = 'MODO DEMO (TRAMA REAL)';
      statusText.textContent = 'Simulación en tiempo real (1 Hz · Datos NEO-6M)';

      this.telemetry = normalizeTelemetry(DEFAULT_TELEMETRY);
      this.lastAnchorUtcMs = Date.parse(this.telemetry.time.utcIso);
      this._applyTelemetry(this.telemetry);

      this.pollIntervalId = setInterval(() => {
        this.lastPacketTimestamp = performance.now();
        this.telemetry.time.iTOW += 1000;
        this.telemetry.time.certifiedEpochs += 1;
        this.lastAnchorUtcMs += 1000;
        this._applyTelemetry(this.telemetry);
      }, 1000);
    }

    _disconnectTransports() {
      if (this.pollIntervalId) {
        clearInterval(this.pollIntervalId);
        this.pollIntervalId = null;
      }
      if (this.ws) {
        try {
          this.ws.onmessage = null;
          this.ws.onerror = null;
          if (this.ws.readyState === WebSocket.OPEN) {
            this.ws.close();
          }
        } catch (_e) {}
        this.ws = null;
      }
      if (this.sse) {
        try { this.sse.close(); } catch (_e) {}
        this.sse = null;
      }
    }

    /**
     * Actualiza la barra de progreso de 10.0s y hace avanzar segundo a segundo
     * el Reloj Atómico UTC y Local entre cada paquete de 10s del ESP32-S3.
     */
    _startCountdownAndClockLoop() {
      const fillEl = document.getElementById('countdown-fill');
      const labelEl = document.getElementById('countdown-label');
      const utcClockEl = document.getElementById('hud-utc-clock');
      const localClockEl = document.getElementById('hud-local-clock');

      const formatIsoSecond = (ms, isUtc) => {
        const d = new Date(ms);
        const yyyy = d.getUTCFullYear();
        const mm = String(d.getUTCMonth() + 1).padStart(2, '0');
        const dd = String(d.getUTCDate()).padStart(2, '0');
        const hh = String(d.getUTCHours()).padStart(2, '0');
        const mi = String(d.getUTCMinutes()).padStart(2, '0');
        const ss = String(d.getUTCSeconds()).padStart(2, '0');
        return `${yyyy}-${mm}-${dd} ${hh}:${mi}:${ss}${isUtc ? 'Z' : ''}`;
      };

      const tick = () => {
        const nowPerf = performance.now();
        const elapsedSincePacket = nowPerf - this.lastPacketTimestamp;
        const cycleElapsed = elapsedSincePacket % this.cyclePeriodMs;
        const remainingSec = Math.max(0, (this.cyclePeriodMs - cycleElapsed) / 1000.0);
        const pct = Math.min(100, Math.max(0, ((this.cyclePeriodMs - cycleElapsed) / this.cyclePeriodMs) * 100));

        if (fillEl) fillEl.style.width = `${pct.toFixed(1)}%`;
        if (labelEl) labelEl.textContent = `Próximo ciclo: ${remainingSec.toFixed(1)}s`;

        // Interpolar el reloj atómico segundo a segundo entre los snapshots de 10s
        if (this.lastAnchorUtcMs && !isNaN(this.lastAnchorUtcMs)) {
          const currentUtcMs = this.lastAnchorUtcMs + Math.min(elapsedSincePacket, 15000);
          const currentLocalMs = currentUtcMs - 5 * 3600 * 1000; // UTC-5 Colombia
          if (utcClockEl) utcClockEl.textContent = formatIsoSecond(currentUtcMs, true);
          if (localClockEl) localClockEl.textContent = formatIsoSecond(currentLocalMs, false);
        }

        requestAnimationFrame(tick);
      };
      requestAnimationFrame(tick);
    }

    /**
     * Aplica un paquete JSON normalizado (secciones [A] a [G]) a la escena 3D y al HUD
     */
    _applyTelemetry(normData) {
      const data = normData.position ? normData : normalizeTelemetry(normData);
      this.telemetry = data;

      const pos  = data.position;
      const kin  = data.kinematics;
      const geo  = data.geometry;
      const tim  = data.time;
      const ion  = data.ionosphere;
      const hw   = data.hardware;
      const sats = data.satellites;

      // 1. Actualizar escena 3D
      this.scene3d.updateTelemetry(data);

      // 2. Actualizar Astrofísica Solar / Lunar y Atmósfera 3D
      this._updateAstronomyFromCurrentState();

      // 3. Actualizar métricas de tiempo de armado de trama y latencia de red
      const frameBuildEl = document.getElementById('val-frame-build');
      if (frameBuildEl) {
        frameBuildEl.textContent =
          `${(data.epochBurstMs || 98.4).toFixed(1)} ms (UBX ${(data.epochUbxMs || 34.2).toFixed(1)} ms · ${data.epochFrames || 18} trm)`;
      }
      const netRttEl = document.getElementById('val-net-rtt');
      if (netRttEl) {
        const rttStr = this.lastHttpRttMs > 0 ? `${this.lastHttpRttMs.toFixed(1)} ms` : '<5 ms';
        netRttEl.textContent = `JSON ${data.jsonBuildUs || 480} µs | Red ${rttStr}`;
      }

      // 4. Actualizar Barra Superior y Subtítulo Geodésico
      const latSign = pos.lat >= 0 ? '+' : '';
      const lonSign = pos.lon >= 0 ? '+' : '';
      const locLabel = this.isLiveConnected ? 'Receptor GNSS Activo' : 'Estación GNSS (Modo Demo)';
      document.getElementById('location-subtitle').textContent =
        `${locLabel} (${latSign}${pos.lat.toFixed(7)}°, ${lonSign}${pos.lon.toFixed(7)}° · ${pos.hMSL.toFixed(1)} m s.n.m.)`;

      document.getElementById('hud-utc-clock').textContent = tim.utcIso.replace('T', ' ');
      document.getElementById('hud-local-clock').textContent = tim.localIso.replace('T', ' ');
      document.getElementById('hud-tacc').textContent = `${tim.tAccNs} ns`;

      // 4. Sección [A] Posición y [B] Cinemática
      const fixBadge = document.getElementById('hud-fix-badge');
      fixBadge.textContent = `FIX ${geo.fixType}D (${pos.navStat}) · ÉPOCAS ${tim.certifiedEpochs}`;

      document.getElementById('val-lat').textContent = `${latSign}${pos.lat.toFixed(7)}°`;
      document.getElementById('val-lon').textContent = `${lonSign}${pos.lon.toFixed(7)}°`;
      document.getElementById('val-alt').textContent =
        `${pos.hMSL.toFixed(2)} m / ${pos.height.toFixed(2)} m`;
      document.getElementById('val-geoid').textContent = `+${pos.geoidSep.toFixed(3)} m`;
      document.getElementById('val-acc').textContent =
        `${pos.hAcc.toFixed(2)} m / ${pos.vAcc.toFixed(2)} m`;
      document.getElementById('val-std').textContent =
        `${pos.stdLat.toFixed(1)} / ${pos.stdLon.toFixed(1)} / ${pos.stdAlt.toFixed(1)} m`;

      const kmh = (kin.gSpeed * 3.6).toFixed(2);
      document.getElementById('val-speed').textContent =
        `${kin.gSpeed.toFixed(3)} m/s (${kmh} km/h)`;
      document.getElementById('val-ned').textContent =
        `${kin.velN >= 0 ? '+' : ''}${kin.velN.toFixed(2)} / ${kin.velE >= 0 ? '+' : ''}${kin.velE.toFixed(2)} / ${kin.velD >= 0 ? '+' : ''}${kin.velD.toFixed(2)} m/s`;
      document.getElementById('val-ecef').textContent =
        `X: +${pos.ecefX.toFixed(2)} m | Y: ${pos.ecefY.toFixed(2)} m | Z: +${pos.ecefZ.toFixed(2)} m`;

      // 5. Sección [C] DOP, [D] Cristal, [E] Ionosfera y [G] UART/Antena
      document.getElementById('val-ttff').textContent =
        `TTFF: ${(geo.ttff / 1000.0).toFixed(2)} s`;
      document.getElementById('dop-g').textContent = geo.gdop.toFixed(2);
      document.getElementById('dop-p').textContent = geo.pdop.toFixed(2);
      document.getElementById('dop-h').textContent = geo.hdop.toFixed(2);
      document.getElementById('dop-v').textContent = geo.vdop.toFixed(2);
      document.getElementById('dop-t').textContent = geo.tdop.toFixed(2);
      document.getElementById('dop-n').textContent = geo.ndop.toFixed(2);
      document.getElementById('dop-e').textContent = geo.edop.toFixed(2);

      document.getElementById('val-clk').textContent =
        `${tim.clkBiasNs} ns | ${tim.clkDriftPpb} ppb`;
      document.getElementById('val-qerr').textContent =
        `${tim.qErrPs >= 0 ? '+' : ''}${tim.qErrPs} ps | Wk ${tim.gpsWeek} (LS ${tim.leapS}s)`;
      document.getElementById('val-raim').textContent =
        `${geo.raimErrLat.toFixed(1)} / ${geo.raimErrLon.toFixed(1)} / ${geo.raimErrAlt.toFixed(1)} m (SV=${geo.raimFailedSV})`;

      document.getElementById('val-uart').textContent =
        `ANT=${hw.antStatus} | Err=${hw.uartErrors} | Salud=${ion.healthMask}`;

      // 6. Sección [F] Bóveda Satelital: Contadores, Skyplot Polar 2D, Barras C/N0 y Tabla
      document.getElementById('count-used').textContent = `${geo.numSVUsed} Usados`;
      document.getElementById('count-view').textContent = `${geo.numSVView} Vista`;

      this._renderCnoBars(sats);
      this._renderSatTable(sats);
      this._draw2DSkyplot(sats, this.lastAstroState);
    }

    _updateAstronomyFromCurrentState() {
      const pos = this.telemetry.position || {};
      const tim = this.telemetry.time || {};
      const lat = pos.lat || 0.0;
      const lon = pos.lon || 0.0;

      let baseUtcDate = new Date(tim.utcIso || '2026-11-09T15:57:03Z');
      if (isNaN(baseUtcDate.getTime())) {
        baseUtcDate = new Date();
      }

      const simDisplay = document.getElementById('sim-time-display');
      const slider = document.getElementById('time-slider');

      if (this.useManualTimeOverride) {
        const hh = Math.floor(this.manualLocalMinutes / 60);
        const mm = this.manualLocalMinutes % 60;
        baseUtcDate = new Date(Date.UTC(
          baseUtcDate.getUTCFullYear(),
          baseUtcDate.getUTCMonth(),
          baseUtcDate.getUTCDate(),
          hh + 5,
          mm,
          0
        ));
        simDisplay.textContent =
          `${String(hh).padStart(2, '0')}:${String(mm).padStart(2, '0')} (Simulación Manual)`;
      } else {
        const localStr = tim.localIso || '2026-11-09T10:57:03';
        const match = localStr.match(/T(\d{2}):(\d{2})/);
        if (match) {
          const hh = parseInt(match[1], 10);
          const mm = parseInt(match[2], 10);
          this.manualLocalMinutes = hh * 60 + mm;
          if (slider) slider.value = String(this.manualLocalMinutes);
          simDisplay.textContent = `${match[1]}:${match[2]} (Tiempo GPS Real)`;
        }
      }

      const astro = global.ObservatorioAstronomy.computeCelestialState(baseUtcDate, lat, lon, -300);
      this.lastAstroState = astro;
      this.scene3d.updateAstronomy(astro);

      document.getElementById('hud-sky-phase').textContent = astro.regimeLabel;
      document.getElementById('hud-sun-pos').textContent =
        `Az: ${astro.sun.azim.toFixed(1)}° | Elev: ${astro.sun.elev >= 0 ? '+' : ''}${astro.sun.elev.toFixed(1)}°`;
      document.getElementById('hud-sun-times').textContent =
        `Orto: ${astro.sun.sunrise} | Ocaso: ${astro.sun.sunset}`;

      document.getElementById('hud-moon-icon').textContent = astro.moon.icon;
      document.getElementById('hud-moon-phase-name').textContent = astro.moon.phaseName;
      document.getElementById('hud-moon-pos').textContent =
        `Az: ${astro.moon.azim.toFixed(1)}° | Elev: ${astro.moon.elev >= 0 ? '+' : ''}${astro.moon.elev.toFixed(1)}°`;
      document.getElementById('hud-moon-illum').textContent =
        `Iluminación: ${(astro.moon.fraction * 100).toFixed(1)}%`;

      if (this.telemetry && this.telemetry.satellites) {
        this._draw2DSkyplot(this.telemetry.satellites, astro);
      }
    }

    _renderCnoBars(sats) {
      const container = document.getElementById('cno-bars-container');
      container.innerHTML = '';

      sats.forEach((sat) => {
        const prefix = sat.type === 'SBAS' ? 'S' : 'G';
        const pct = Math.min(100, Math.max(4, ((sat.cno || 0) / 50.0) * 100));
        const cls = sat.elev < 0 ? 'below' : sat.used ? 'used' : 'view';

        const col = document.createElement('div');
        col.className = 'cno-col';
        col.title = `${sat.type} PRN ${sat.prn} · C/N0=${sat.cno} dB-Hz · Elev=${sat.elev}° Az=${sat.azim}°`;
        col.innerHTML = `
          <div class="cno-val">${sat.cno}</div>
          <div class="cno-track">
            <div class="cno-fill ${cls}" style="height: ${pct.toFixed(0)}%;"></div>
          </div>
          <div class="cno-prn">${prefix}${sat.prn}</div>
        `;
        col.addEventListener('click', () => {
          this.selectedPrn = sat.prn;
          this.scene3d.focusSatelliteByPrn(sat.prn);
          this._highlightTableRow(sat.prn);
        });
        container.appendChild(col);
      });
    }

    _renderSatTable(sats) {
      const tbody = document.getElementById('sat-table-body');
      tbody.innerHTML = '';

      sats.forEach((sat) => {
        const tr = document.createElement('tr');
        tr.setAttribute('data-prn', String(sat.prn));
        if (this.selectedPrn === sat.prn) tr.classList.add('selected');

        const isBelow = sat.elev < 0;
        const statusCell = isBelow
          ? '<span class="accent-rose">BAJO &lt;0°</span>'
          : sat.used
          ? '<span class="accent-emerald">USADO</span>'
          : '<span class="accent-amber">VISTA</span>';

        const grsStr =
          sat.grsResM === null || sat.grsResM === undefined
            ? 'N/A'
            : `${sat.grsResM >= 0 ? '+' : ''}${Number(sat.grsResM).toFixed(1)}m`;

        tr.innerHTML = `
          <td><strong>${sat.prn}</strong></td>
          <td>${sat.type}</td>
          <td>${statusCell}</td>
          <td>${sat.cno}</td>
          <td class="${isBelow ? 'accent-rose' : ''}">${sat.elev >= 0 ? '+' : ''}${sat.elev}°</td>
          <td>${sat.azim}°</td>
          <td>${sat.prResCm >= 0 ? '+' : ''}${sat.prResCm}cm</td>
          <td>${grsStr}</td>
        `;

        tr.addEventListener('click', () => {
          this.selectedPrn = sat.prn;
          this.scene3d.focusSatelliteByPrn(sat.prn);
          this._highlightTableRow(sat.prn);
        });

        tbody.appendChild(tr);
      });
    }

    _highlightTableRow(prn) {
      document.querySelectorAll('#sat-table-body tr').forEach((row) => {
        if (parseInt(row.getAttribute('data-prn'), 10) === prn) {
          row.classList.add('selected');
        } else {
          row.classList.remove('selected');
        }
      });
    }

    /**
     * Dibuja el radar acimutal cenital 2D en el panel derecho (#skyplot-2d)
     */
    _draw2DSkyplot(sats, astro) {
      const canvas = document.getElementById('skyplot-2d');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;
      const cx = w / 2;
      const cy = h / 2;
      const maxR = cx - 22;

      ctx.clearRect(0, 0, w, h);

      const bgGrad = ctx.createRadialGradient(cx, cy, 4, cx, cy, maxR);
      bgGrad.addColorStop(0, 'rgba(14, 28, 54, 0.92)');
      bgGrad.addColorStop(1, 'rgba(4, 9, 20, 0.96)');
      ctx.fillStyle = bgGrad;
      ctx.beginPath();
      ctx.arc(cx, cy, maxR, 0, Math.PI * 2);
      ctx.fill();

      [0, 30, 60].forEach((el) => {
        const r = maxR * (1 - el / 90.0);
        ctx.strokeStyle = el === 0 ? 'rgba(56, 189, 248, 0.55)' : 'rgba(148, 163, 184, 0.22)';
        ctx.lineWidth = el === 0 ? 1.5 : 1;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();
      });

      ctx.strokeStyle = 'rgba(148, 163, 184, 0.22)';
      ctx.beginPath();
      ctx.moveTo(cx, cy - maxR); ctx.lineTo(cx, cy + maxR);
      ctx.moveTo(cx - maxR, cy); ctx.lineTo(cx + maxR, cy);
      ctx.stroke();

      ctx.fillStyle = '#38bdf8';
      ctx.font = 'bold 11px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('N', cx, cy - maxR - 11);
      ctx.fillStyle = '#94a3b8';
      ctx.fillText('S', cx, cy + maxR + 11);
      ctx.fillText('E', cx + maxR + 11, cy);
      ctx.fillText('O', cx - maxR - 11, cy);

      const projectPolar = (azDeg, elDeg) => {
        const clampedEl = Math.max(0, Math.min(90, elDeg));
        const r = maxR * (1.0 - clampedEl / 90.0);
        const azRad = (azDeg * Math.PI) / 180.0;
        return {
          x: cx + r * Math.sin(azRad),
          y: cy - r * Math.cos(azRad)
        };
      };

      if (astro && astro.sun && astro.sun.elev >= 0) {
        const sp = projectPolar(astro.sun.azim, astro.sun.elev);
        ctx.fillStyle = '#facc15';
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, 5.5, 0, Math.PI * 2);
        ctx.fill();
      }
      if (astro && astro.moon && astro.moon.elev >= 0) {
        const mp = projectPolar(astro.moon.azim, astro.moon.elev);
        ctx.fillStyle = '#e2e8f0';
        ctx.beginPath();
        ctx.arc(mp.x, mp.y, 4.5, 0, Math.PI * 2);
        ctx.fill();
      }

      sats.forEach((sat) => {
        let pt;
        if (sat.elev < 0) {
          const azRad = ((sat.azim || 0) * Math.PI) / 180.0;
          pt = {
            x: cx + (maxR + 6) * Math.sin(azRad),
            y: cy - (maxR + 6) * Math.cos(azRad)
          };
        } else {
          pt = projectPolar(sat.azim || 0, sat.elev || 0);
        }

        ctx.fillStyle = sat.elev < 0 ? '#fb7185' : sat.used ? '#10b981' : '#f59e0b';
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, sat.used ? 4.5 : 3.5, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#f8fafc';
        ctx.font = '600 9px JetBrains Mono, monospace';
        ctx.fillText(String(sat.prn), pt.x, pt.y - 8);
      });
    }
  }

  window.addEventListener('DOMContentLoaded', () => {
    global.observatorioApp = new ObservatorioApp();
  });
})(window);
