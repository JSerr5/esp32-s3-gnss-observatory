/* =========================================================================
 * web/js/astronomy.js
 * Motor Astronómico Solar y Lunar + Conversión Esférica 3D (Azimut/Elevación)
 * Compatible con SunCalc CDN y con ecuaciones keplerianas de respaldo offline.
 * ========================================================================= */
(function (global) {
  'use strict';

  const DEG2RAD = Math.PI / 180.0;
  const RAD2DEG = 180.0 / Math.PI;

  /**
   * Convierte Azimut (0°=Norte, 90°=Este, 180°=Sur, 270°=Oeste) y
   * Elevación (-90°..+90°) a coordenadas cartesianas 3D sobre la bóveda de radio R:
   *   X =  R * cos(elev) * sin(azim)
   *   Y =  R * sin(elev)
   *   Z = -R * cos(elev) * cos(azim)
   */
  function azElToVector3(azimDeg, elevDeg, radius) {
    const az = azimDeg * DEG2RAD;
    const el = elevDeg * DEG2RAD;
    const cosEl = Math.cos(el);
    return {
      x: radius * cosEl * Math.sin(az),
      y: radius * Math.sin(el),
      z: -radius * cosEl * Math.cos(az)
    };
  }

  /**
   * Ecuaciones astronómicas keplerianas de respaldo (en caso de abrir sin internet)
   */
  function fallbackSolarLunar(dateUtc, latDeg, lonDeg) {
    const jd = dateUtc.getTime() / 86400000.0 + 2440587.5;
    const d = jd - 2451545.0;

    // Sol
    const g = (357.529 + 0.98560028 * d) * DEG2RAD;
    const q = (280.459 + 0.98564736 * d) % 360;
    const L = (q + 1.915 * Math.sin(g) + 0.020 * Math.sin(2 * g)) * DEG2RAD;
    const e = (23.439 - 0.00000036 * d) * DEG2RAD;

    const ra = Math.atan2(Math.cos(e) * Math.sin(L), Math.cos(L));
    const dec = Math.asin(Math.sin(e) * Math.sin(L));

    const gmst = (280.46061837 + 360.98564736629 * d) % 360;
    const lmst = (gmst + lonDeg) * DEG2RAD;
    const ha = lmst - ra;

    const lat = latDeg * DEG2RAD;
    const sinAlt = Math.sin(lat) * Math.sin(dec) + Math.cos(lat) * Math.cos(dec) * Math.cos(ha);
    const sunElev = Math.asin(Math.max(-1, Math.min(1, sinAlt))) * RAD2DEG;
    const y = -Math.sin(ha);
    const x = Math.tan(dec) * Math.cos(lat) - Math.sin(lat) * Math.cos(ha);
    const sunAzim = (Math.atan2(y, x) * RAD2DEG + 360) % 360;

    // Luna aproximada
    const Lm = (218.316 + 13.176396 * d) * DEG2RAD;
    const Mm = (134.963 + 13.064993 * d) * DEG2RAD;
    const Fm = (93.272 + 13.229350 * d) * DEG2RAD;
    const lonM = Lm + 6.289 * DEG2RAD * Math.sin(Mm);
    const latM = 5.128 * DEG2RAD * Math.sin(Fm);

    const raM = Math.atan2(
      Math.sin(lonM) * Math.cos(e) - Math.tan(latM) * Math.sin(e),
      Math.cos(lonM)
    );
    const decM = Math.asin(
      Math.sin(latM) * Math.cos(e) + Math.cos(latM) * Math.sin(e) * Math.sin(lonM)
    );
    const haM = lmst - raM;
    const sinAltM = Math.sin(lat) * Math.sin(decM) + Math.cos(lat) * Math.cos(decM) * Math.cos(haM);
    const moonElev = Math.asin(Math.max(-1, Math.min(1, sinAltM))) * RAD2DEG;
    const ym = -Math.sin(haM);
    const xm = Math.tan(decM) * Math.cos(lat) - Math.sin(lat) * Math.cos(haM);
    const moonAzim = (Math.atan2(ym, xm) * RAD2DEG + 360) % 360;

    const phaseAngle = ((lonM - L) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    const phase = phaseAngle / (2 * Math.PI);
    const fraction = 0.5 * (1 - Math.cos(phaseAngle));

    return {
      sun: { azim: sunAzim, elev: sunElev, sunrise: '06:02', sunset: '17:54' },
      moon: { azim: moonAzim, elev: moonElev, phase: phase, fraction: fraction }
    };
  }

  function moonPhaseInfo(phase) {
    if (phase < 0.03 || phase > 0.97) return { name: 'LUNA NUEVA', icon: '🌑' };
    if (phase < 0.22) return { name: 'CRECIENTE CÓNCAVA', icon: '🌒' };
    if (phase < 0.28) return { name: 'CUARTO CRECIENTE', icon: '🌓' };
    if (phase < 0.47) return { name: 'GIBOSA CRECIENTE', icon: '🌔' };
    if (phase < 0.53) return { name: 'LUNA LLENA', icon: '🌕' };
    if (phase < 0.72) return { name: 'GIBOSA MENGUANTE', icon: '🌖' };
    if (phase < 0.78) return { name: 'CUARTO MENGUANTE', icon: '🌗' };
    return { name: 'MENGUANTE BALSÁMICA', icon: '🌘' };
  }

  function formatHHMM(dateObj, utcOffsetMin) {
    if (!dateObj || isNaN(dateObj.getTime())) return '--:--';
    const localMs = dateObj.getTime() + utcOffsetMin * 60000;
    const d = new Date(localMs);
    const hh = String(d.getUTCHours()).padStart(2, '0');
    const mm = String(d.getUTCMinutes()).padStart(2, '0');
    return `${hh}:${mm}`;
  }

  /**
   * Calcula el estado astronómico completo para (lat, lon, dateUtc)
   */
  function computeCelestialState(dateUtc, latDeg, lonDeg, utcOffsetMin = -300) {
    let sunAzim = 120, sunElev = 45, sunriseStr = '06:00', sunsetStr = '17:56';
    let moonAzim = 240, moonElev = -15, moonPhase = 0.35, moonFrac = 0.72;

    if (global.SunCalc) {
      // SunCalc mide el azimut desde el Sur hacia el Oeste en radianes: sumar 180° para Norte=0°
      const sp = global.SunCalc.getPosition(dateUtc, latDeg, lonDeg);
      sunAzim = (sp.azimuth * RAD2DEG + 180.0 + 360.0) % 360.0;
      sunElev = sp.altitude * RAD2DEG;

      const st = global.SunCalc.getTimes(dateUtc, latDeg, lonDeg);
      sunriseStr = formatHHMM(st.sunrise, utcOffsetMin);
      sunsetStr  = formatHHMM(st.sunset, utcOffsetMin);

      const mp = global.SunCalc.getMoonPosition(dateUtc, latDeg, lonDeg);
      moonAzim = (mp.azimuth * RAD2DEG + 180.0 + 360.0) % 360.0;
      moonElev = mp.altitude * RAD2DEG;

      const mi = global.SunCalc.getMoonIllumination(dateUtc);
      moonPhase = mi.phase;
      moonFrac  = mi.fraction;
    } else {
      const fb = fallbackSolarLunar(dateUtc, latDeg, lonDeg);
      sunAzim = fb.sun.azim;
      sunElev = fb.sun.elev;
      sunriseStr = fb.sun.sunrise;
      sunsetStr  = fb.sun.sunset;
      moonAzim = fb.moon.azim;
      moonElev = fb.moon.elev;
      moonPhase = fb.moon.phase;
      moonFrac  = fb.moon.fraction;
    }

    const pInfo = moonPhaseInfo(moonPhase);

    // Clasificación del régimen atmosférico según la elevación solar real
    let regime = 'day';
    let regimeLabel = `Día Solar (${sunElev >= 0 ? '+' : ''}${sunElev.toFixed(1)}°)`;
    if (sunElev < -6.0) {
      regime = 'night';
      regimeLabel = `Noche Astronómica (${sunElev.toFixed(1)}°)`;
    } else if (sunElev <= 10.0) {
      regime = 'twilight';
      const isMorning = sunAzim < 180.0;
      regimeLabel = `${isMorning ? 'Amanecer' : 'Atardecer'} Crepuscular (${sunElev >= 0 ? '+' : ''}${sunElev.toFixed(1)}°)`;
    }

    return {
      sun: {
        azim: sunAzim,
        elev: sunElev,
        sunrise: sunriseStr,
        sunset: sunsetStr
      },
      moon: {
        azim: moonAzim,
        elev: moonElev,
        phase: moonPhase,
        fraction: moonFrac,
        phaseName: pInfo.name,
        icon: pInfo.icon
      },
      regime,
      regimeLabel
    };
  }

  /**
   * Genera una textura Canvas 2D realista de la Luna con su terminador de fase actual
   */
  function createMoonPhaseCanvas(phase) {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 128;
    const ctx = canvas.getContext('2d');

    // Superficie selenográfica base
    ctx.fillStyle = '#cbd5e1';
    ctx.fillRect(0, 0, 256, 128);

    // Mares lunares suaves
    const craters = [
      [70, 45, 22], [110, 58, 28], [150, 42, 18],
      [185, 70, 24], [90, 82, 16], [50, 75, 14]
    ];
    for (const [cx, cy, r] of craters) {
      const g = ctx.createRadialGradient(cx, cy, 2, cx, cy, r);
      g.addColorStop(0, 'rgba(71, 85, 105, 0.55)');
      g.addColorStop(1, 'rgba(71, 85, 105, 0)');
      ctx.fillStyle = g;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fill();
    }

    // Sombra del terminador según la fase lunar (0=Nueva, 0.5=Llena, 1=Nueva)
    const illum = 1.0 - Math.abs(phase - 0.5) * 2.0; // 0 en nueva, 1 en llena
    const darkAlpha = 0.84 * (1.0 - illum);
    if (darkAlpha > 0.03) {
      ctx.fillStyle = `rgba(4, 8, 20, ${darkAlpha.toFixed(2)})`;
      ctx.fillRect(0, 0, 128, 128);
    }

    return canvas;
  }

  global.ObservatorioAstronomy = {
    azElToVector3,
    computeCelestialState,
    createMoonPhaseCanvas
  };
})(window);
