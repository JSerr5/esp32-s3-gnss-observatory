/* =========================================================================
 * web/js/scene3d.js
 * Motor Gráfico 3D (Three.js): Bóveda Celeste, Mapa Cartográfico Real,
 * Elipsoide 3D de Incertidumbre, Constelación GNSS/SBAS, Sol, Luna,
 * Atmósfera Rayleigh/Mie, Cámara Libre 3D + Ojo de Pez y Raycasting.
 * ========================================================================= */
(function (global) {
  'use strict';

  const DOME_RADIUS = 260;       // Radio de la bóveda celeste (unidades 3D = metros escénicos)
  const GROUND_RADIUS = 260;     // Radio del disco cartográfico en el plano Y = 0
  const DEG2RAD = Math.PI / 180.0;

  class ObservatorioScene3D {
    constructor(containerEl, labelsLayerEl, tooltipEl) {
      this.container = containerEl;
      this.labelsLayer = labelsLayerEl;
      this.tooltip = tooltipEl;

      this.width = window.innerWidth;
      this.height = window.innerHeight;
      this.cameraMode = 'orbit'; // 'orbit' | 'fisheye-up' | 'fisheye-top'
      this.ellipsoidScale = 2.0;
      this.mapProvider = 'esri';
      this.lastTileKey = '';

      this.satMeshes = [];       // Objetos interactivos para Raycasting
      this.satGroup = null;
      this.laserGroup = null;
      this.labelItems = [];      // { pos3d: THREE.Vector3, el: HTMLElement, minElev: number }

      this.hoveredSat = null;
      this.pinnedSatPrn = null;
      this.onSatelliteSelect = null;

      this.pointer = new THREE.Vector2(-999, -999);
      this.pointerScreen = { x: 0, y: 0 };
      this.raycaster = new THREE.Raycaster();

      this.lastTelemetry = null;
      this.lastAstro = null;
      this.clock = new THREE.Clock();

      this._initEngine();
      this._initSkyAndStars();
      this._initLights();
      this._initGroundAndMonument();
      this._initCelestialDome();
      this._initSunAndMoon();
      this._initInteraction();
      this._animate();
    }

    _initEngine() {
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x040814);
      this.scene.fog = new THREE.FogExp2(0x091326, 0.00055);

      this.camera = new THREE.PerspectiveCamera(55, this.width / this.height, 0.5, 3000);
      this.camera.position.set(0, 145, 310);

      this.renderer = new THREE.WebGLRenderer({
        antialias: true,
        powerPreference: 'high-performance'
      });
      this.renderer.setSize(this.width, this.height);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 0.95;

      this.container.innerHTML = '';
      this.container.appendChild(this.renderer.domElement);

      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.06;
      this.controls.maxDistance = 850;
      this.controls.minDistance = 4;
      this.controls.target.set(0, 18, 0);
      this.controls.update();
    }

    _initSkyAndStars() {
      // Shader atmosférico físico Rayleigh / Mie (Three.js Sky)
      if (THREE.Sky) {
        this.sky = new THREE.Sky();
        this.sky.scale.setScalar(2200);
        this.scene.add(this.sky);

        const uniforms = this.sky.material.uniforms;
        uniforms['turbidity'].value = 4.5;
        uniforms['rayleigh'].value = 2.2;
        uniforms['mieCoefficient'].value = 0.005;
        uniforms['mieDirectionalG'].value = 0.82;
      }

      // Bóveda estelar para atardecer y noche (2500 estrellas con magnitud variable)
      const starCount = 2500;
      const starGeo = new THREE.BufferGeometry();
      const positions = new Float32Array(starCount * 3);
      const colors = new Float32Array(starCount * 3);

      for (let i = 0; i < starCount; i++) {
        const theta = Math.random() * Math.PI * 2;
        const phi = Math.acos(Math.random() * 0.98 + 0.01); // Hemisferio superior + horizonte
        const r = 1350;
        positions[i * 3]     = r * Math.sin(phi) * Math.cos(theta);
        positions[i * 3 + 1] = r * Math.cos(phi);
        positions[i * 3 + 2] = r * Math.sin(phi) * Math.sin(theta);

        const tint = 0.78 + Math.random() * 0.22;
        colors[i * 3]     = tint;
        colors[i * 3 + 1] = tint;
        colors[i * 3 + 2] = Math.min(1.0, tint + 0.08);
      }

      starGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      starGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

      this.starMat = new THREE.PointsMaterial({
        size: 2.4,
        vertexColors: true,
        transparent: true,
        opacity: 0.15,
        depthWrite: false
      });

      this.starField = new THREE.Points(starGeo, this.starMat);
      this.scene.add(this.starField);
    }

    _initLights() {
      this.hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 0.55);
      this.scene.add(this.hemiLight);

      this.ambientLight = new THREE.AmbientLight(0x334155, 0.45);
      this.scene.add(this.ambientLight);

      // Luz direccional Solar con sombras proyectadas sobre el mapa
      this.sunLight = new THREE.DirectionalLight(0xfffbeb, 1.35);
      this.sunLight.position.set(120, 220, -90);
      this.sunLight.castShadow = true;
      this.sunLight.shadow.mapSize.width = 2048;
      this.sunLight.shadow.mapSize.height = 2048;
      this.sunLight.shadow.camera.near = 10;
      this.sunLight.shadow.camera.far = 750;
      const d = 240;
      this.sunLight.shadow.camera.left = -d;
      this.sunLight.shadow.camera.right = d;
      this.sunLight.shadow.camera.top = d;
      this.sunLight.shadow.camera.bottom = -d;
      this.scene.add(this.sunLight);

      // Luz direccional Lunar tenue para la noche
      this.moonLight = new THREE.DirectionalLight(0x93c5fd, 0.25);
      this.moonLight.position.set(-140, 180, 100);
      this.scene.add(this.moonLight);
    }

    _createRadarFallbackTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 1024;
      canvas.height = 1024;
      const ctx = canvas.getContext('2d');

      const grad = ctx.createRadialGradient(512, 512, 20, 512, 512, 512);
      grad.addColorStop(0, '#0b192e');
      grad.addColorStop(0.65, '#071020');
      grad.addColorStop(1, '#030712');
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, 1024, 1024);

      ctx.strokeStyle = 'rgba(56, 189, 248, 0.14)';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 1024; i += 64) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, 1024); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(1024, i); ctx.stroke();
      }

      ctx.strokeStyle = 'rgba(16, 185, 129, 0.24)';
      for (const r of [120, 240, 360, 480]) {
        ctx.beginPath();
        ctx.arc(512, 512, r, 0, Math.PI * 2);
        ctx.stroke();
      }

      const tex = new THREE.CanvasTexture(canvas);
      tex.anisotropy = 4;
      return tex;
    }

    _initGroundAndMonument() {
      // Disco cartográfico en el plano del horizonte (Y = 0)
      const groundGeo = new THREE.CircleGeometry(GROUND_RADIUS, 96);
      groundGeo.rotateX(-Math.PI / 2);

      this.groundMat = new THREE.MeshStandardMaterial({
        map: this._createRadarFallbackTexture(),
        roughness: 0.86,
        metalness: 0.08
      });

      this.groundMesh = new THREE.Mesh(groundGeo, this.groundMat);
      this.groundMesh.receiveShadow = true;
      this.scene.add(this.groundMesh);

      // Anillo exterior metálico del horizonte (Elevación = 0°)
      const rimGeo = new THREE.RingGeometry(GROUND_RADIUS - 1.4, GROUND_RADIUS + 1.4, 128);
      rimGeo.rotateX(-Math.PI / 2);
      const rimMat = new THREE.MeshBasicMaterial({
        color: 0x38bdf8,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.55
      });
      const rimMesh = new THREE.Mesh(rimGeo, rimMat);
      rimMesh.position.y = 0.08;
      this.scene.add(rimMesh);

      // Anillos métricos de distancia sobre el terreno (10m, 25m, 50m, 100m, 200m)
      this.metricRingsGroup = new THREE.Group();
      const radii = [15, 35, 75, 140, 210];
      radii.forEach((r) => {
        const ringGeo = new THREE.RingGeometry(r - 0.25, r + 0.25, 96);
        ringGeo.rotateX(-Math.PI / 2);
        const ringMat = new THREE.MeshBasicMaterial({
          color: 0x38bdf8,
          transparent: true,
          opacity: 0.22,
          side: THREE.DoubleSide
        });
        const m = new THREE.Mesh(ringGeo, ringMat);
        m.position.y = 0.06;
        this.metricRingsGroup.add(m);
      });
      this.scene.add(this.metricRingsGroup);

      // Monumento central del receptor u-blox NEO-6M en el origen (0, 0, 0)
      this.receiverGroup = new THREE.Group();

      const pedestalGeo = new THREE.CylinderGeometry(1.4, 2.2, 3.2, 24);
      const pedestalMat = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        metalness: 0.65,
        roughness: 0.3
      });
      const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
      pedestal.position.y = 1.6;
      pedestal.castShadow = true;
      this.receiverGroup.add(pedestal);

      // Antena cerámica patch activa sobre el pedestal
      const patchGeo = new THREE.BoxGeometry(2.6, 0.7, 2.6);
      const patchMat = new THREE.MeshStandardMaterial({
        color: 0x10b981,
        emissive: 0x059669,
        emissiveIntensity: 0.55,
        roughness: 0.25,
        metalness: 0.4
      });
      const patch = new THREE.Mesh(patchGeo, patchMat);
      patch.position.y = 3.4;
      patch.castShadow = true;
      this.receiverGroup.add(patch);

      // Pulso luminoso cenital en el receptor
      const beaconGeo = new THREE.SphereGeometry(0.75, 16, 16);
      const beaconMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
      this.receiverBeacon = new THREE.Mesh(beaconGeo, beaconMat);
      this.receiverBeacon.position.y = 4.2;
      this.receiverGroup.add(this.receiverBeacon);

      this.scene.add(this.receiverGroup);

      // Elipsoide 3D semitransparente de incertidumbre (stdLat, stdLon, stdAlt / hAcc, vAcc)
      const ellipGeo = new THREE.SphereGeometry(1.0, 36, 28);
      const ellipMat = new THREE.MeshPhysicalMaterial({
        color: 0x38bdf8,
        emissive: 0x0284c7,
        emissiveIntensity: 0.22,
        transparent: true,
        opacity: 0.20,
        roughness: 0.15,
        metalness: 0.05,
        depthWrite: false,
        side: THREE.DoubleSide
      });
      this.errorEllipsoid = new THREE.Mesh(ellipGeo, ellipMat);
      this.errorEllipsoid.position.set(0, 0.2, 0);

      // Malla alámbrica envolvente del elipsoide 1-sigma
      const wireGeo = new THREE.WireframeGeometry(new THREE.SphereGeometry(1.003, 16, 12));
      const wireMat = new THREE.LineBasicMaterial({
        color: 0x38bdf8,
        transparent: true,
        opacity: 0.38
      });
      this.errorEllipsoidWire = new THREE.LineSegments(wireGeo, wireMat);
      this.errorEllipsoid.add(this.errorEllipsoidWire);
      this.scene.add(this.errorEllipsoid);

      // Flecha 3D del vector velocidad local NED (velN, velE, velD)
      this.velocityArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, -1),
        new THREE.Vector3(0, 3.8, 0),
        12,
        0x10b981,
        3.2,
        1.8
      );
      this.scene.add(this.velocityArrow);
    }

    /**
     * Descarga y ensambla un mosaico 3x3 Web Mercator (Zoom 17) centrado exactamente
     * en (lat, lon) con desplazamiento sub-píxel para que (0,0,0) coincida con la
     * posición exacta del receptor.
     */
    updateGroundMap(latDeg, lonDeg, provider = this.mapProvider) {
      this.mapProvider = provider;
      const zoom = 17;
      const key = `${provider}:${latDeg.toFixed(5)}:${lonDeg.toFixed(5)}:${zoom}`;
      if (key === this.lastTileKey) return;
      this.lastTileKey = key;

      const n = Math.pow(2, zoom);
      const latRad = latDeg * DEG2RAD;
      const exactX = ((lonDeg + 180.0) / 360.0) * n;
      const exactY =
        ((1.0 - Math.log(Math.tan(latRad) + 1.0 / Math.cos(latRad)) / Math.PI) / 2.0) * n;

      const centerTileX = Math.floor(exactX);
      const centerTileY = Math.floor(exactY);
      const fracX = exactX - centerTileX; // 0..1 dentro del tile central
      const fracY = exactY - centerTileY;

      const tileCanvas = document.createElement('canvas');
      tileCanvas.width = 768;
      tileCanvas.height = 768;
      const ctx = tileCanvas.getContext('2d');

      // Fondo táctico inmediato mientras llegan los tiles
      ctx.fillStyle = '#091526';
      ctx.fillRect(0, 0, 768, 768);

      let loadedCount = 0;
      const getTileUrl = (tx, ty) => {
        if (provider === 'osm') {
          return `https://tile.openstreetmap.org/${zoom}/${tx}/${ty}.png`;
        }
        if (provider === 'dark') {
          return `https://a.basemaps.cartocdn.com/dark_all/${zoom}/${tx}/${ty}.png`;
        }
        // Por defecto: Esri World Imagery (Satelital de alta resolución)
        return `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${zoom}/${ty}/${tx}`;
      };

      // Desplazamiento para que la coordenada exacta (fracX, fracY) quede en el centro (384, 384)
      const offsetX = Math.round((0.5 - fracX) * 256);
      const offsetY = Math.round((0.5 - fracY) * 256);

      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const tx = centerTileX + dx;
          const ty = centerTileY + dy;
          const img = new Image();
          img.crossOrigin = 'anonymous';
          img.onload = () => {
            const px = (dx + 1) * 256 + offsetX;
            const py = (dy + 1) * 256 + offsetY;
            ctx.drawImage(img, px, py, 256, 256);
            loadedCount++;

            if (loadedCount === 9 || loadedCount % 3 === 0) {
              // Retícula sutil y mira geodésica central
              ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
              ctx.lineWidth = 1.5;
              ctx.beginPath();
              ctx.arc(384, 384, 18, 0, Math.PI * 2);
              ctx.stroke();

              const tex = new THREE.CanvasTexture(tileCanvas);
              tex.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
              this.groundMat.map = tex;
              this.groundMat.needsUpdate = true;
            }
          };
          img.src = getTileUrl(tx, ty);
        }
      }
    }

    _initCelestialDome() {
      this.domeGroup = new THREE.Group();

      // 1. Anillos de elevación constante: 0° (Horizonte), 30°, 60° y 85° (Cenit)
      const elevations = [0, 30, 60, 85];
      elevations.forEach((elDeg) => {
        const elRad = elDeg * DEG2RAD;
        const r = DOME_RADIUS * Math.cos(elRad);
        const y = DOME_RADIUS * Math.sin(elRad);

        const pts = [];
        const segments = 128;
        for (let i = 0; i <= segments; i++) {
          const theta = (i / segments) * Math.PI * 2;
          pts.push(new THREE.Vector3(r * Math.sin(theta), y, -r * Math.cos(theta)));
        }
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        const mat = new THREE.LineBasicMaterial({
          color: elDeg === 0 ? 0x38bdf8 : 0x64748b,
          transparent: true,
          opacity: elDeg === 0 ? 0.65 : 0.30
        });
        this.domeGroup.add(new THREE.Line(geo, mat));
      });

      // 2. Meridianos acimutales cada 30° desde el horizonte hasta el Cenit (90°)
      for (let azDeg = 0; azDeg < 360; azDeg += 30) {
        const isCardinal = azDeg % 90 === 0;
        const pts = [];
        for (let elDeg = 0; elDeg <= 90; elDeg += 3) {
          const v = global.ObservatorioAstronomy.azElToVector3(azDeg, elDeg, DOME_RADIUS);
          pts.push(new THREE.Vector3(v.x, v.y, v.z));
        }
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        const mat = new THREE.LineBasicMaterial({
          color: azDeg === 0 ? 0x38bdf8 : isCardinal ? 0x94a3b8 : 0x475569,
          transparent: true,
          opacity: isCardinal ? 0.45 : 0.18
        });
        this.domeGroup.add(new THREE.Line(geo, mat));
      }

      // 3. Ejes Cardinales sobre el suelo (Norte=-Z, Este=+X, Sur=+Z, Oeste=-X)
      const cardinals = [
        { label: 'N', az: 0, color: 0x38bdf8 },
        { label: 'E', az: 90, color: 0x94a3b8 },
        { label: 'S', az: 180, color: 0x94a3b8 },
        { label: 'O', az: 270, color: 0x94a3b8 }
      ];

      cardinals.forEach((c) => {
        const v = global.ObservatorioAstronomy.azElToVector3(c.az, 0, DOME_RADIUS);
        const lineGeo = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0, 0.15, 0),
          new THREE.Vector3(v.x, 0.15, v.z)
        ]);
        const lineMat = new THREE.LineBasicMaterial({
          color: c.color,
          transparent: true,
          opacity: c.az === 0 ? 0.75 : 0.35
        });
        this.domeGroup.add(new THREE.Line(lineGeo, lineMat));

        // Etiqueta cardinal flotante
        const badge = document.createElement('div');
        badge.className = 'label-3d cardinal';
        badge.textContent = c.label;
        this.labelsLayer.appendChild(badge);
        const pos = global.ObservatorioAstronomy.azElToVector3(c.az, 2.5, DOME_RADIUS * 1.02);
        this.labelItems.push({
          pos3d: new THREE.Vector3(pos.x, pos.y, pos.z),
          el: badge,
          alwaysVisible: true
        });
      });

      // Etiquetas de anillos de elevación (30°, 60°, Cenit 90°)
      [30, 60, 90].forEach((elDeg) => {
        const badge = document.createElement('div');
        badge.className = 'label-3d';
        badge.style.opacity = '0.65';
        badge.textContent = elDeg === 90 ? 'CENIT 90°' : `ELEV ${elDeg}°`;
        this.labelsLayer.appendChild(badge);
        const v = global.ObservatorioAstronomy.azElToVector3(15, elDeg, DOME_RADIUS);
        this.labelItems.push({
          pos3d: new THREE.Vector3(v.x, v.y, v.z),
          el: badge,
          alwaysVisible: true
        });
      });

      this.scene.add(this.domeGroup);

      // Contenedores dinámicos para satélites y rayos láser de pseudorrango
      this.satGroup = new THREE.Group();
      this.laserGroup = new THREE.Group();
      this.scene.add(this.satGroup);
      this.scene.add(this.laserGroup);
    }

    _createGlowTexture(colorHexInner, colorHexOuter) {
      const canvas = document.createElement('canvas');
      canvas.width = 128;
      canvas.height = 128;
      const ctx = canvas.getContext('2d');
      const g = ctx.createRadialGradient(64, 64, 4, 64, 64, 62);
      g.addColorStop(0, colorHexInner);
      g.addColorStop(0.35, colorHexOuter);
      g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, 128, 128);
      return new THREE.CanvasTexture(canvas);
    }

    _initSunAndMoon() {
      // SOL 3D con corona luminosa
      this.sunGroup = new THREE.Group();
      const sunGeo = new THREE.SphereGeometry(9.5, 32, 32);
      const sunMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });
      this.sunMesh = new THREE.Mesh(sunGeo, sunMat);
      this.sunGroup.add(this.sunMesh);

      const sunGlowMat = new THREE.SpriteMaterial({
        map: this._createGlowTexture('rgba(254, 240, 138, 1)', 'rgba(245, 158, 11, 0.45)'),
        blending: THREE.AdditiveBlending,
        transparent: true,
        depthWrite: false
      });
      const sunSprite = new THREE.Sprite(sunGlowMat);
      sunSprite.scale.set(52, 52, 1);
      this.sunGroup.add(sunSprite);
      this.scene.add(this.sunGroup);

      this.sunLabel = document.createElement('div');
      this.sunLabel.className = 'label-3d celestial';
      this.sunLabel.textContent = '☀ SOL';
      this.labelsLayer.appendChild(this.sunLabel);
      this.sunLabelPos = new THREE.Vector3();
      this.labelItems.push({ pos3d: this.sunLabelPos, el: this.sunLabel, alwaysVisible: true });

      // LUNA 3D con textura selenográfica y fase real
      this.moonGroup = new THREE.Group();
      const moonGeo = new THREE.SphereGeometry(7.2, 32, 32);
      const moonCanvas = global.ObservatorioAstronomy.createMoonPhaseCanvas(0.35);
      this.moonTex = new THREE.CanvasTexture(moonCanvas);
      this.moonMat = new THREE.MeshStandardMaterial({
        map: this.moonTex,
        roughness: 0.85,
        metalness: 0.05,
        emissive: 0x334155,
        emissiveIntensity: 0.35
      });
      this.moonMesh = new THREE.Mesh(moonGeo, this.moonMat);
      this.moonGroup.add(this.moonMesh);

      const moonGlowMat = new THREE.SpriteMaterial({
        map: this._createGlowTexture('rgba(226, 232, 240, 0.8)', 'rgba(148, 163, 184, 0.25)'),
        blending: THREE.AdditiveBlending,
        transparent: true,
        depthWrite: false
      });
      const moonSprite = new THREE.Sprite(moonGlowMat);
      moonSprite.scale.set(32, 32, 1);
      this.moonGroup.add(moonSprite);
      this.scene.add(this.moonGroup);

      this.moonLabel = document.createElement('div');
      this.moonLabel.className = 'label-3d celestial';
      this.moonLabel.textContent = '☾ LUNA';
      this.labelsLayer.appendChild(this.moonLabel);
      this.moonLabelPos = new THREE.Vector3();
      this.labelItems.push({ pos3d: this.moonLabelPos, el: this.moonLabel, alwaysVisible: true });
    }

    /**
     * Actualiza la posición del Sol, la Luna, las sombras y el shader atmosférico
     * según la efeméride astronómica calculada para (lat, lon, fecha/hora).
     */
    updateAstronomy(astroState) {
      this.lastAstro = astroState;
      const { sun, moon, regime } = astroState;

      // Posición 3D del Sol sobre la bóveda
      const sunPos = global.ObservatorioAstronomy.azElToVector3(
        sun.azim,
        sun.elev,
        DOME_RADIUS * 0.96
      );
      this.sunGroup.position.set(sunPos.x, sunPos.y, sunPos.z);
      this.sunLabelPos.set(sunPos.x, sunPos.y + 13, sunPos.z);
      this.sunLabel.textContent = `☀ SOL (${sun.elev >= 0 ? '+' : ''}${sun.elev.toFixed(1)}°)`;
      this.sunLabel.style.opacity = sun.elev >= -12 ? '1' : '0.35';

      // Posición de la luz direccional solar
      if (sun.elev > -4) {
        this.sunLight.position.set(sunPos.x, Math.max(12, sunPos.y), sunPos.z);
      } else {
        this.sunLight.position.set(0, -100, 0);
      }

      // Posición 3D de la Luna sobre la bóveda
      const moonPos = global.ObservatorioAstronomy.azElToVector3(
        moon.azim,
        moon.elev,
        DOME_RADIUS * 0.92
      );
      this.moonGroup.position.set(moonPos.x, moonPos.y, moonPos.z);
      this.moonLabelPos.set(moonPos.x, moonPos.y + 11, moonPos.z);
      this.moonLabel.textContent = `${moon.icon} LUNA (${moon.elev >= 0 ? '+' : ''}${moon.elev.toFixed(1)}°)`;

      // Actualizar textura de fase lunar si cambió significativamente
      if (this._lastMoonPhase === undefined || Math.abs(this._lastMoonPhase - moon.phase) > 0.02) {
        this._lastMoonPhase = moon.phase;
        const c = global.ObservatorioAstronomy.createMoonPhaseCanvas(moon.phase);
        this.moonMat.map = new THREE.CanvasTexture(c);
        this.moonMat.needsUpdate = true;
      }

      if (moon.elev > 0) {
        this.moonLight.position.set(moonPos.x, moonPos.y, moonPos.z);
        this.moonLight.intensity = 0.22 + 0.25 * moon.fraction;
      } else {
        this.moonLight.intensity = 0.05;
      }

      // Shader atmosférico Rayleigh / Mie y estrellas según elevación solar:
      //   - Día (elev > 10°)
      //   - Amanecer / Atardecer (-6° <= elev <= 10°)
      //   - Noche (elev < -6°)
      if (this.sky) {
        const uniforms = this.sky.material.uniforms;
        uniforms['sunPosition'].value.set(sunPos.x, sunPos.y, sunPos.z);

        if (regime === 'day') {
          uniforms['turbidity'].value = 4.0;
          uniforms['rayleigh'].value = 2.0;
          this.renderer.toneMappingExposure = 0.95;
          this.sunLight.intensity = 1.45;
          this.sunLight.color.setHex(0xfffbeb);
          this.hemiLight.intensity = 0.65;
          this.starMat.opacity = 0.0;
          this.scene.fog.color.setHex(0x1e3a5f);
        } else if (regime === 'twilight') {
          uniforms['turbidity'].value = 10.5;
          uniforms['rayleigh'].value = 3.6;
          this.renderer.toneMappingExposure = 0.72;
          this.sunLight.intensity = 0.85;
          this.sunLight.color.setHex(0xfb923c); // Tonos cálidos crepusculares
          this.hemiLight.intensity = 0.38;
          this.starMat.opacity = 0.48;
          this.scene.fog.color.setHex(0x1e1b4b);
        } else {
          // Noche astronómica
          uniforms['turbidity'].value = 1.5;
          uniforms['rayleigh'].value = 0.18;
          this.renderer.toneMappingExposure = 0.42;
          this.sunLight.intensity = 0.0;
          this.hemiLight.intensity = 0.18;
          this.starMat.opacity = 0.95;
          this.scene.fog.color.setHex(0x040814);
        }
      }
    }

    /**
     * Construye el modelo 3D de un satélite (cuerpo + paneles solares + halo)
     */
    _buildSatelliteMesh(sat) {
      const group = new THREE.Group();
      const isBelowHorizon = sat.elev < 0;
      const isUsed = Boolean(sat.used);

      let primaryColor = 0xf59e0b; // En vista (Ámbar)
      let glowColorInner = 'rgba(245, 158, 11, 0.95)';
      let glowColorOuter = 'rgba(245, 158, 11, 0.25)';

      if (isBelowHorizon) {
        primaryColor = 0xfb7185; // Bajo el horizonte (< 0°), ej. PRN 131 SBAS (-91°)
        glowColorInner = 'rgba(251, 113, 133, 0.95)';
        glowColorOuter = 'rgba(244, 63, 94, 0.28)';
      } else if (isUsed) {
        primaryColor = 0x10b981; // Usado en solución Fix 3D (Esmeralda/Cian)
        glowColorInner = 'rgba(56, 189, 248, 0.98)';
        glowColorOuter = 'rgba(16, 185, 129, 0.35)';
      }

      // Cuerpo central del satélite (Bus orbital)
      const bodyGeo = new THREE.OctahedronGeometry(isUsed ? 3.4 : 2.6, 1);
      const bodyMat = new THREE.MeshStandardMaterial({
        color: primaryColor,
        emissive: primaryColor,
        emissiveIntensity: isUsed ? 0.65 : 0.35,
        roughness: 0.25,
        metalness: 0.7
      });
      const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
      group.add(bodyMesh);

      // Paneles solares laterales desplegados
      const wingGeo = new THREE.BoxGeometry(10.5, 0.25, 2.2);
      const wingMat = new THREE.MeshStandardMaterial({
        color: isUsed ? 0x0284c7 : 0x334155,
        emissive: isUsed ? 0x0369a1 : 0x1e293b,
        emissiveIntensity: 0.35,
        metalness: 0.8,
        roughness: 0.2
      });
      const wings = new THREE.Mesh(wingGeo, wingMat);
      group.add(wings);

      // Halo luminoso proporcional a la potencia C/N0 (dB-Hz)
      const cnoScale = 12 + Math.max(0, Math.min(50, sat.cno || 0)) * 0.32;
      const spriteMat = new THREE.SpriteMaterial({
        map: this._createGlowTexture(glowColorInner, glowColorOuter),
        blending: THREE.AdditiveBlending,
        transparent: true,
        depthWrite: false
      });
      const halo = new THREE.Sprite(spriteMat);
      halo.scale.set(cnoScale, cnoScale, 1);
      group.add(halo);

      // Esfera invisible ligeramente mayor para facilitar el Raycasting con el cursor
      const hitGeo = new THREE.SphereGeometry(6.8, 12, 12);
      const hitMat = new THREE.MeshBasicMaterial({ visible: false });
      const hitMesh = new THREE.Mesh(hitGeo, hitMat);
      hitMesh.userData.sat = sat;
      hitMesh.userData.parentGroup = group;
      group.add(hitMesh);

      return { group, hitMesh };
    }

    /**
     * Actualiza toda la escena 3D con un nuevo paquete de telemetría JSON del ESP32-S3
     */
    updateTelemetry(data) {
      this.lastTelemetry = data;
      const pos = data.position || {};
      const kin = data.kinematics || {};
      const sats = data.satellites || [];

      // 1. Actualizar mapa cartográfico real centrado en (lat, lon)
      if (typeof pos.lat === 'number' && typeof pos.lon === 'number') {
        this.updateGroundMap(pos.lat, pos.lon, this.mapProvider);
      }

      // 2. Actualizar elipsoide 3D de incertidumbre (stdLon = Este X, stdAlt = Arriba Y, stdLat = Norte Z)
      const sx = Math.max(1.5, (pos.stdLon || pos.hAcc || 10) * this.ellipsoidScale);
      const sy = Math.max(1.5, (pos.stdAlt || pos.vAcc || 12) * this.ellipsoidScale);
      const sz = Math.max(1.5, (pos.stdLat || pos.hAcc || 10) * this.ellipsoidScale);
      this.errorEllipsoid.scale.set(sx, sy, sz);

      // 3. Actualizar vector velocidad 3D NED (Norte=-Z, Este=+X, Arriba=-velD=+Y)
      const vN = kin.velN || 0;
      const vE = kin.velE || 0;
      const vU = -(kin.velD || 0);
      const vVec = new THREE.Vector3(vE, vU, -vN);
      const speedMag = vVec.length();
      if (speedMag > 0.02) {
        this.velocityArrow.visible = true;
        this.velocityArrow.setDirection(vVec.clone().normalize());
        const arrowLen = Math.min(65, Math.max(10, speedMag * 45));
        this.velocityArrow.setLength(arrowLen, 3.2, 1.8);
      } else {
        this.velocityArrow.visible = false;
      }

      // 4. Limpiar satélites, líneas láser y etiquetas 2D anteriores de satélites
      while (this.satGroup.children.length > 0) {
        this.satGroup.remove(this.satGroup.children[0]);
      }
      while (this.laserGroup.children.length > 0) {
        this.laserGroup.remove(this.laserGroup.children[0]);
      }
      this.satMeshes = [];
      this.labelItems = this.labelItems.filter((item) => {
        if (item.isSatLabel) {
          if (item.el && item.el.parentNode) item.el.parentNode.removeChild(item.el);
          return false;
        }
        return true;
      });

      // 5. Posicionar cada satélite del arreglo [F] sobre la bóveda esférica:
      //    X = R * cos(elev) * sin(azim)
      //    Y = R * sin(elev)
      //    Z = -R * cos(elev) * cos(azim)
      sats.forEach((sat) => {
        const v = global.ObservatorioAstronomy.azElToVector3(
          sat.azim || 0,
          sat.elev || 0,
          DOME_RADIUS
        );
        const satPos = new THREE.Vector3(v.x, v.y, v.z);

        const { group, hitMesh } = this._buildSatelliteMesh(sat);
        group.position.copy(satPos);
        group.lookAt(0, 0, 0);
        this.satGroup.add(group);
        this.satMeshes.push(hitMesh);

        // Si el satélite es usado en la solución (used = true), trazar línea láser cian al receptor (0,0,0)
        if (sat.used) {
          const lineGeo = new THREE.BufferGeometry().setFromPoints([
            satPos,
            new THREE.Vector3(0, 3.8, 0)
          ]);
          const lineMat = new THREE.LineDashedMaterial({
            color: 0x38bdf8,
            dashSize: 6,
            gapSize: 3,
            transparent: true,
            opacity: 0.72
          });
          const laserLine = new THREE.Line(lineGeo, lineMat);
          laserLine.computeLineDistances();
          this.laserGroup.add(laserLine);
        } else if (sat.elev < 0) {
          // Si tiene elevación negativa (ej. PRN 131 SBAS a -91°), trazar línea punteada roja al plano horizonte Y=0
          const projGeo = new THREE.BufferGeometry().setFromPoints([
            satPos,
            new THREE.Vector3(satPos.x, 0, satPos.z)
          ]);
          const projMat = new THREE.LineDashedMaterial({
            color: 0xfb7185,
            dashSize: 4,
            gapSize: 4,
            transparent: true,
            opacity: 0.55
          });
          const projLine = new THREE.Line(projGeo, projMat);
          projLine.computeLineDistances();
          this.laserGroup.add(projLine);
        }

        // Etiqueta flotante 2D proyectada
        const tag = document.createElement('div');
        const prefix = sat.type === 'SBAS' ? 'S' : 'G';
        const stateCls = sat.elev < 0 ? 'sat-below' : sat.used ? 'sat-used' : 'sat-view';
        tag.className = `label-3d ${stateCls}`;
        tag.textContent = `${prefix}${String(sat.prn).padStart(2, '0')} · ${sat.cno}dB`;
        this.labelsLayer.appendChild(tag);

        this.labelItems.push({
          pos3d: satPos.clone().add(new THREE.Vector3(0, 6.5, 0)),
          el: tag,
          isSatLabel: true,
          prn: sat.prn
        });
      });

      // Si había un satélite fijado en el tooltip, refrescar sus datos
      if (this.pinnedSatPrn !== null) {
        const updated = sats.find((s) => s.prn === this.pinnedSatPrn);
        if (updated) {
          this._renderTooltip(updated, this.pointerScreen.x, this.pointerScreen.y, true);
        }
      }
    }

    setEllipsoidScale(scaleVal) {
      this.ellipsoidScale = parseFloat(scaleVal) || 2.0;
      if (this.lastTelemetry) {
        this.updateTelemetry(this.lastTelemetry);
      }
    }

    /**
     * Cambia entre los modos de cámara:
     *  - 'orbit': Cámara libre 3D con OrbitControls alrededor del receptor.
     *  - 'fisheye-up': Vista Ojo de Pez desde el suelo (0, 2.2, 0) mirando al Cenit (90° arriba, Norte arriba).
     *  - 'fisheye-top': Domo Polar Acimutal desde el Cenit mirando hacia abajo (0, 560, 0).
     */
    setCameraMode(mode) {
      this.cameraMode = mode;
      const canvasEl = this.renderer.domElement;

      if (mode === 'fisheye-up') {
        // Ojo de Pez desde la posición del receptor mirando hacia la bóveda celeste (90° arriba)
        this.camera.fov = 118;
        this.camera.updateProjectionMatrix();
        this.camera.position.set(0, 2.5, 0.01);
        this.camera.up.set(0, 0, -1); // Norte (-Z) hacia arriba de la pantalla
        this.controls.target.set(0, DOME_RADIUS, 0);
        this.controls.minDistance = 1;
        this.controls.maxDistance = 300;
        this.controls.update();
        canvasEl.style.clipPath = 'circle(47% at 50% 50%)';
      } else if (mode === 'fisheye-top') {
        // Proyección Polar Cenital Superior (Mirando desde arriba del Cenit hacia el suelo)
        this.camera.fov = 54;
        this.camera.updateProjectionMatrix();
        this.camera.position.set(0, 560, 0.01);
        this.camera.up.set(0, 0, -1); // Norte (-Z) hacia arriba de la pantalla
        this.controls.target.set(0, 0, 0);
        this.controls.minDistance = 80;
        this.controls.maxDistance = 900;
        this.controls.update();
        canvasEl.style.clipPath = 'none';
      } else {
        // Órbita 3D libre predeterminada
        this.camera.fov = 55;
        this.camera.updateProjectionMatrix();
        this.camera.up.set(0, 1, 0);
        this.camera.position.set(0, 145, 310);
        this.controls.target.set(0, 18, 0);
        this.controls.minDistance = 4;
        this.controls.maxDistance = 850;
        this.controls.update();
        canvasEl.style.clipPath = 'none';
      }
    }

    /**
     * Enfoca suavemente un satélite específico en el cielo 3D al hacer clic en la tabla [F]
     */
    focusSatelliteByPrn(prn) {
      const hit = this.satMeshes.find((m) => m.userData.sat && m.userData.sat.prn === prn);
      if (!hit) return;

      const sat = hit.userData.sat;
      const grp = hit.userData.parentGroup;
      this.pinnedSatPrn = prn;

      // Proyectar su posición 3D a pantalla para posicionar el tooltip
      const projected = grp.position.clone().project(this.camera);
      const sx = ((projected.x + 1) * this.width) / 2;
      const sy = ((-projected.y + 1) * this.height) / 2;
      this.pointerScreen = { x: sx, y: sy };
      this._renderTooltip(sat, sx, sy, true);

      if (this.cameraMode === 'orbit') {
        this.controls.target.copy(grp.position.clone().multiplyScalar(0.35));
        this.controls.update();
      }
    }

    _initInteraction() {
      window.addEventListener('resize', () => {
        this.width = window.innerWidth;
        this.height = window.innerHeight;
        this.camera.aspect = this.width / this.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.width, this.height);
      });

      const dom = this.renderer.domElement;

      dom.addEventListener('pointermove', (ev) => {
        const rect = dom.getBoundingClientRect();
        this.pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
        this.pointer.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
        this.pointerScreen = { x: ev.clientX, y: ev.clientY };

        this.raycaster.setFromCamera(this.pointer, this.camera);
        const hits = this.raycaster.intersectObjects(this.satMeshes, false);

        if (hits.length > 0) {
          const sat = hits[0].object.userData.sat;
          this.hoveredSat = sat;
          dom.style.cursor = 'pointer';
          this._renderTooltip(sat, ev.clientX, ev.clientY, this.pinnedSatPrn === sat.prn);
        } else {
          this.hoveredSat = null;
          dom.style.cursor = 'grab';
          if (this.pinnedSatPrn === null) {
            this.tooltip.classList.add('hidden');
          }
        }
      });

      dom.addEventListener('click', (ev) => {
        this.raycaster.setFromCamera(this.pointer, this.camera);
        const hits = this.raycaster.intersectObjects(this.satMeshes, false);
        if (hits.length > 0) {
          const sat = hits[0].object.userData.sat;
          this.pinnedSatPrn = sat.prn;
          this._renderTooltip(sat, ev.clientX, ev.clientY, true);
          if (typeof this.onSatelliteSelect === 'function') {
            this.onSatelliteSelect(sat.prn);
          }
        } else {
          this.pinnedSatPrn = null;
          this.tooltip.classList.add('hidden');
          if (typeof this.onSatelliteSelect === 'function') {
            this.onSatelliteSelect(null);
          }
        }
      });
    }

    _renderTooltip(sat, clientX, clientY, isPinned = false) {
      if (!sat) return;
      const isBelow = sat.elev < 0;
      const badgeClass = isBelow ? 'below' : sat.used ? 'used' : 'view';
      const badgeText = isBelow
        ? 'BAJO HORIZONTE (<0°)'
        : sat.used
        ? 'USADO EN FIX 3D'
        : 'EN VISTA (NO USADO)';

      const prResMeters = ((sat.prResCm || 0) / 100.0).toFixed(2);
      const grsText =
        sat.grsResM === null || sat.grsResM === undefined
          ? 'N/A'
          : `${sat.grsResM >= 0 ? '+' : ''}${Number(sat.grsResM).toFixed(1)} m`;

      this.tooltip.innerHTML = `
        <div class="tt-header">
          <span class="tt-title">${sat.type} PRN ${sat.prn} (CH ${sat.chn})</span>
          <span class="tt-badge ${badgeClass}">${badgeText}</span>
        </div>
        <div class="tt-grid">
          <div><span class="tt-k">Potencia C/N0:</span> <span class="tt-v">${sat.cno} dB-Hz</span></div>
          <div><span class="tt-k">Calidad UBX:</span> <span class="tt-v">QI=${sat.qi} (${sat.orb || 'N/A'})</span></div>
          <div><span class="tt-k">Elevación:</span> <span class="tt-v">${sat.elev >= 0 ? '+' : ''}${sat.elev}°</span></div>
          <div><span class="tt-k">Azimut:</span> <span class="tt-v">${sat.azim}°</span></div>
          <div><span class="tt-k">Residual SVINFO:</span> <span class="tt-v">${sat.prResCm >= 0 ? '+' : ''}${sat.prResCm} cm (${prResMeters} m)</span></div>
          <div><span class="tt-k">Residual GPGRS:</span> <span class="tt-v">${grsText}</span></div>
          <div><span class="tt-k">Salud SV:</span> <span class="tt-v">${sat.healthy ? 'SANO (OK)' : 'NO SANO'}</span></div>
          <div><span class="tt-k">Corrección DGPS:</span> <span class="tt-v">${sat.dgps ? 'SÍ' : 'NO'}</span></div>
        </div>
      `;

      const pad = 16;
      const maxX = Math.min(window.innerWidth - 280, clientX + pad);
      const maxY = Math.min(window.innerHeight - 190, clientY + pad);
      this.tooltip.style.left = `${Math.max(12, maxX)}px`;
      this.tooltip.style.top = `${Math.max(68, maxY)}px`;
      this.tooltip.classList.remove('hidden');
    }

    _updateProjectedLabels() {
      const halfW = this.width / 2;
      const halfH = this.height / 2;

      for (const item of this.labelItems) {
        const v = item.pos3d.clone().project(this.camera);
        if (v.z > 1.0 || v.z < -1.0) {
          item.el.style.display = 'none';
          continue;
        }
        const x = v.x * halfW + halfW;
        const y = -v.y * halfH + halfH;
        if (x < -40 || x > this.width + 40 || y < 52 || y > this.height - 32) {
          item.el.style.display = 'none';
          continue;
        }
        item.el.style.display = 'block';
        item.el.style.left = `${x}px`;
        item.el.style.top = `${y}px`;
      }
    }

    _animate() {
      requestAnimationFrame(() => this._animate());

      const t = this.clock.getElapsedTime();
      if (this.receiverBeacon) {
        const s = 1.0 + 0.25 * Math.sin(t * 4.0);
        this.receiverBeacon.scale.set(s, s, s);
      }

      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this._updateProjectedLabels();
    }
  }

  global.ObservatorioScene3D = ObservatorioScene3D;
})(window);
