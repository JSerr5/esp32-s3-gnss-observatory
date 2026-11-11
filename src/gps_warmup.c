/*
 * gps_warmup.c
 * ---------------------------------------------------------------------------
 * Agente 4 - Implementacion de la Maquina de Estados de Calentamiento (Fases
 * A..E), seguimiento de descarga orbital (efemerides / almanaque / C/N0) y
 * Filtro Cientifico de Confianza de 9 criterios durante 5 epocas consecutivas.
 * ---------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include "gps_config.h"
#include "gps_warmup.h"

const char *gps_warmup_quality_str(uint8_t q)
{
    switch (q & 0x07) {
    case 0:  return "0:Inactivo";
    case 1:  return "1:Buscando";
    case 2:  return "2:Adquirida";
    case 3:  return "3:Detectada(no util)";
    case 4:  return "4:Code Lock";
    case 5:  return "5:Carrier Lock";
    case 6:  return "6:Carrier Lock+";
    case 7:  return "7:Carrier+Datos 50bps";
    default: return "?:Desconocido";
    }
}

static void set_phase(gps_repo_t *r, gps_phase_t ph, uint32_t now_ms)
{
    if (r->phase != ph) {
        r->phase = ph;
        r->phase_since_ms = now_ms;
    }
}

/* -------------------------------------------------------------------------
 * Cruza NAV-SVINFO, RXM-SVSI, AID-EPH, AID-ALM, GPGSV y PUBX,03 para
 * contabilizar el progreso real de enganche y descarga orbital.
 * ------------------------------------------------------------------------- */
void gps_warmup_refresh_channels(gps_repo_t *r)
{
    uint8_t total = 0, idle = 0, search = 0, acq = 0, det = 0;
    uint8_t code = 0, carrier = 0, cno_pos = 0, strong = 0, used = 0;
    uint8_t eph = 0, alm = 0, healthy = 0, vis = 0, sbas = 0;

    if (r->svinfo.valid) {
        total = r->svinfo.n;
        for (uint8_t i = 0; i < r->svinfo.n; i++) {
            const ubx_nav_svinfo_blk_t *b = &r->svinfo.b[i];
            uint8_t q = (uint8_t)(b->quality & 0x0F);
            switch (q) {
            case 0: idle++;    break;
            case 1: search++;  break;
            case 2: acq++;     break;
            case 3: det++;     break;
            case 4: code++;    break;
            default: carrier++; break;
            }
            if (b->cno > 0) {
                cno_pos++;
            }
            if (b->cno >= WARMUP_EPH_CNO_THRESHOLD) {
                strong++;
            }
            if (b->flags & 0x01) {
                used++;
            }
            if (b->flags & 0x08) {
                eph++;
            }
            if (b->flags & 0x20) {
                alm++;
            }
            if ((b->flags & 0x10) == 0 && b->cno > 0) {
                healthy++;
            }
            if (b->elev >= 0 && b->svid != 0) {
                vis++;
            }
            if (b->svid >= 120 && b->svid <= 158 && b->cno > 0) {
                sbas++;
            }
        }
    } else if (r->pubx03.valid) {
        total = (uint8_t)r->pubx03.d.nblk;
        vis   = total;
        for (int i = 0; i < r->pubx03.d.nblk; i++) {
            const nmea_pubx03_sat_t *s = &r->pubx03.d.sat[i];
            if (s->cno > 0) cno_pos++;
            if (s->cno >= WARMUP_EPH_CNO_THRESHOLD) strong++;
            if (s->status == 'U') { used++; eph++; carrier++; }
            else if (s->status == 'e') { eph++; code++; }
            else if (s->cno > 0) { det++; }
            else { search++; }
            if (s->sv >= 120 && s->sv <= 158 && s->cno > 0) sbas++;
        }
    } else if (r->gsv.valid) {
        vis = (uint8_t)r->gsv.in_view;
        total = (uint8_t)r->gsv.n;
        for (int i = 0; i < r->gsv.n; i++) {
            if (r->gsv.s[i].cno > 0) cno_pos++;
            if (r->gsv.s[i].cno >= WARMUP_EPH_CNO_THRESHOLD) strong++;
        }
        if (r->gsa.valid) {
            used = (uint8_t)r->gsa.d.nprn;
        }
    }

    /* Cruzar con RXM-SVSI si aporta conteo superior de efemerides/almanaque */
    if (r->svsi.valid) {
        uint8_t svsi_eph = 0, svsi_alm = 0, svsi_hlth = 0;
        for (uint8_t i = 0; i < r->svsi.n; i++) {
            uint8_t fl = r->svsi.b[i].svFlag;
            if (fl & 0x20) svsi_eph++;
            if (fl & 0x40) svsi_alm++;
            if (fl & 0x10) svsi_hlth++;
        }
        if (svsi_eph > eph)   eph = svsi_eph;
        if (svsi_alm > alm)   alm = svsi_alm;
        if (svsi_hlth > healthy) healthy = svsi_hlth;
        if (r->svsi.h.numVis > vis) vis = r->svsi.h.numVis;
    }

    /* Cruzar con sondeos directos AID-EPH / AID-ALM (PRN 1..32) */
    uint8_t aid_eph = 0, aid_alm = 0;
    for (int sv = 1; sv <= 32; sv++) {
        if (r->eph_how[sv] != 0)  aid_eph++;
        if (r->alm_week[sv] != 0) aid_alm++;
    }
    if (aid_eph > eph) eph = aid_eph;
    if (aid_alm > alm) alm = aid_alm;

    r->ch_total   = total;
    r->ch_idle    = idle;
    r->ch_search  = search;
    r->ch_acq     = acq;
    r->ch_detect  = det;
    r->ch_code    = code;
    r->ch_carrier = carrier;
    r->sv_cno_pos = cno_pos;
    r->sv_strong  = strong;
    r->sv_used    = used;
    r->sv_eph     = eph;
    r->sv_alm     = alm;
    r->sv_healthy = healthy;
    r->sv_vis     = vis;
    r->sv_sbas    = sbas;
}

/* -------------------------------------------------------------------------
 * Evaluacion de cada epoca PVT sintetizada (Seccion 5: Fases A..E y filtro
 * cientifico de 9 condiciones durante 5 epocas consecutivas).
 * ------------------------------------------------------------------------- */
void gps_warmup_on_epoch(gps_repo_t *r, uint32_t now_ms)
{
    if (r->warm_start_ms == 0) {
        r->warm_start_ms = now_ms;
    }

    gps_warmup_refresh_channels(r);

    const gps_pvt_t *p = &r->pvt;

    /* Filtro fisico del rumbo (Seccion 3.4.b):
     * Cuando gSpeed < 0.10 m/s (10 cm/s), el rumbo por efecto Doppler carece
     * de significado fisico: se congela el ultimo rumbo valido. */
    if (p->gSpeed >= HEADING_MIN_GSPEED_CMS) {
        r->heading_last_valid = p->heading;
        r->heading_have       = true;
        r->heading_static     = false;
    } else {
        r->heading_static     = true;
    }

    /* 1. Evaluar los 9 criterios obligatorios del Filtro de Confianza */
    uint16_t crit = 0;

    uint8_t fix_sol = p->gpsFix_sol;
    uint8_t fix_st  = p->gpsFix_st;
    bool fix_geom_sol = (fix_sol == 2 || fix_sol == 3 || fix_sol == 4);
    bool fix_geom_st  = (fix_st  == 2 || fix_st  == 3 || fix_st  == 4);

    if (fix_geom_sol && fix_geom_st) {
        crit |= CRIT_FIX_TYPE;
    }
    if ((p->flags_sol & 0x01) != 0 && (p->flags_st & 0x01) != 0) {
        crit |= CRIT_FIX_OK;
    }

    /* NMEA GPGGA y GPRMC deben estar activos y frescos */
    bool gga_fresh = r->gga.valid && ((now_ms - r->gga.ms) <= TRUST_NMEA_FRESH_MS);
    if (gga_fresh && r->gga.d.quality >= 1) {
        crit |= CRIT_GGA_Q;
    }

    bool rmc_fresh = r->rmc.valid && ((now_ms - r->rmc.ms) <= TRUST_NMEA_FRESH_MS);
    if (rmc_fresh && r->rmc.d.status == 'A') {
        crit |= CRIT_RMC_A;
    }

    /* Tiempo atomico UTC: validTOW(b0) + validWKN(b1) + validUTC(b2) = 0x07 */
    if ((p->utc_valid & 0x07) == 0x07) {
        crit |= CRIT_UTC_VALID;
    }
    if ((p->utc_valid & 0x07) == 0x07 && p->utc_tAcc < TRUST_MAX_TACC_NS) {
        crit |= CRIT_TACC;
    }

    /* Numero de satelites en la solucion (sin tope de 12) */
    uint8_t req_sv = (fix_sol == 2) ? TRUST_MIN_SV_2D : TRUST_MIN_SV_3D;
    if (p->numSV >= req_sv) {
        crit |= CRIT_NUMSV;
    }

    /* Dilucion de precision PDOP < 10.0 */
    uint16_t pdop_use = (p->pDOP > 0) ? p->pDOP : p->pDOP_sol;
    if (pdop_use > 0 && pdop_use < TRUST_MAX_PDOP_X100) {
        crit |= CRIT_PDOP;
    }

    /* Incertidumbre horizontal hAcc < 50000 mm (50 m) */
    if (p->hAcc > 0 && p->hAcc < TRUST_MAX_HACC_MM) {
        crit |= CRIT_HACC;
    }

    r->crit = crit;

    /* Sincronizacion de tiempo conocida (Fase C) */
    bool tow_wkn_ok = ((p->flags_sol & 0x0C) == 0x0C) ||
                      ((p->flags_st  & 0x0C) == 0x0C) ||
                      ((p->utc_valid & 0x03) == 0x03) ||
                      ((p->tgps_valid & 0x03) == 0x03);
    r->time_known = tow_wkn_ok;
    if ((p->tgps_valid & 0x04) != 0) {
        r->leap_from_space = true;
    }

    /* Primera fijacion geometrica: congelar TTFF oficial y abrir convergencia */
    if ((fix_geom_sol || fix_geom_st) && !r->ttff_frozen) {
        if (p->ttff > 0) {
            r->ttff_ms = p->ttff;
        } else {
            r->ttff_ms = now_ms - r->warm_start_ms;
        }
        r->ttff_local_ms = now_ms - r->warm_start_ms;
        r->ttff_frozen   = true;

        r->hacc_first = p->hAcc;
        r->vacc_first = p->vAcc;
        r->sacc_first = p->sAcc;
        r->pdop_first = pdop_use;
    }

    /* Actualizar minimos de convergencia del filtro Kalman */
    if (fix_geom_sol || fix_geom_st) {
        if (p->hAcc > 0 && p->hAcc < r->hacc_min) r->hacc_min = p->hAcc;
        if (p->vAcc > 0 && p->vAcc < r->vacc_min) r->vacc_min = p->vAcc;
        if (p->sAcc > 0 && p->sAcc < r->sacc_min) r->sacc_min = p->sAcc;
        if (pdop_use > 0 && pdop_use < r->pdop_min) r->pdop_min = pdop_use;
        if (p->hDOP > 0 && p->hDOP < r->hdop_min) r->hdop_min = p->hDOP;
        if (p->vDOP > 0 && p->vDOP < r->vdop_min) r->vdop_min = p->vDOP;
    }

    /* Conteo de epocas consecutivas que cumplen los 9 criterios */
    if (crit == CRIT_ALL_MASK) {
        if (r->good_epochs < 250) {
            r->good_epochs++;
        }
        r->bad_epochs = 0;
    } else {
        r->good_epochs = 0;
        if (r->bad_epochs < 250) {
            r->bad_epochs++;
        }
    }

    /* Transiciones de la Maquina de Estados de Calentamiento (Fases A..E) */
    if (r->phase == PH_E_TRUSTED) {
        /* Histeresis: solo degrada si falla durante TRUST_LOSS_EPOCHS seguidas */
        if (r->bad_epochs >= TRUST_LOSS_EPOCHS) {
            if (fix_geom_sol || fix_geom_st) {
                set_phase(r, PH_D_CONVERGE, now_ms);
            } else if (tow_wkn_ok) {
                set_phase(r, PH_C_TIME, now_ms);
            } else if (r->sv_cno_pos > 0) {
                set_phase(r, PH_B_SIGNALS, now_ms);
            } else {
                set_phase(r, PH_A_BLIND, now_ms);
            }
        }
    } else {
        if (r->good_epochs >= TRUST_REQUIRED_EPOCHS) {
            set_phase(r, PH_E_TRUSTED, now_ms);
            r->certified_ms = now_ms;
            r->cert_count++;
        } else if (fix_geom_sol || fix_geom_st) {
            set_phase(r, PH_D_CONVERGE, now_ms);
        } else if (tow_wkn_ok || fix_sol == 5 || fix_st == 5) {
            set_phase(r, PH_C_TIME, now_ms);
        } else if (r->sv_cno_pos > 0 || r->ch_code > 0 || r->ch_carrier > 0) {
            set_phase(r, PH_B_SIGNALS, now_ms);
        } else {
            set_phase(r, PH_A_BLIND, now_ms);
        }
    }

    /* Deteccion de tiempo prolongado en interiores sin fix */
    uint32_t elapsed_s = (now_ms - r->warm_start_ms) / 1000u;
    r->indoor_timeout = (!r->ttff_frozen && elapsed_s >= WARMUP_INDOOR_TIMEOUT_S);
}

/* -------------------------------------------------------------------------
 * Modo degradado (solo escucha NMEA cuando GPIO 4 TX no esta cableado)
 * ------------------------------------------------------------------------- */
void gps_warmup_on_nmea_epoch(gps_repo_t *r, uint32_t now_ms)
{
    if (r->warm_start_ms == 0) {
        r->warm_start_ms = now_ms;
    }

    gps_warmup_refresh_channels(r);

    uint16_t crit = 0;
    bool gga_ok = r->gga.valid && r->gga.d.quality >= 1;
    bool rmc_ok = r->rmc.valid && r->rmc.d.status == 'A';
    bool gsa_ok = r->gsa.valid && (r->gsa.d.fixmode == 2 || r->gsa.d.fixmode == 3);

    if (gga_ok) crit |= CRIT_GGA_Q;
    if (rmc_ok) crit |= CRIT_RMC_A;
    if (gsa_ok) crit |= (CRIT_FIX_TYPE | CRIT_FIX_OK);
    if (rmc_ok && r->rmc.d.time.valid && r->rmc.d.date.valid) {
        crit |= CRIT_UTC_VALID;
        r->time_known = true;
    }
    int req_sv = (r->gsa.valid && r->gsa.d.fixmode == 2) ? TRUST_MIN_SV_2D : TRUST_MIN_SV_3D;
    if (r->gga.valid && r->gga.d.numSV >= req_sv) {
        crit |= CRIT_NUMSV;
    }
    if (r->gsa.valid && r->gsa.d.pdop_valid && r->gsa.d.pdop < 10.0f) {
        crit |= CRIT_PDOP;
    }
    r->crit = crit;

    /* Filtro de rumbo en NMEA (0.10 m/s = 0.194384 nudos) */
    if (r->rmc.valid && r->rmc.d.sog_valid && r->rmc.d.sog_kn >= 0.194384f) {
        if (r->rmc.d.cog_valid) {
            r->heading_last_valid = (int32_t)(r->rmc.d.cog * 100000.0f);
            r->heading_have       = true;
        }
        r->heading_static = false;
    } else {
        r->heading_static = true;
    }

    if (gga_ok && rmc_ok && !r->ttff_frozen) {
        r->ttff_ms       = now_ms - r->warm_start_ms;
        r->ttff_local_ms = r->ttff_ms;
        r->ttff_frozen   = true;
    }

    if (gga_ok && rmc_ok && gsa_ok) {
        set_phase(r, PH_D_CONVERGE, now_ms);
    } else if (r->time_known) {
        set_phase(r, PH_C_TIME, now_ms);
    } else if (r->sv_cno_pos > 0) {
        set_phase(r, PH_B_SIGNALS, now_ms);
    } else {
        set_phase(r, PH_A_BLIND, now_ms);
    }
}

/* -------------------------------------------------------------------------
 * Formateador explicito de los 9 criterios del Filtro Cientifico de Confianza
 * ------------------------------------------------------------------------- */
bool gps_warmup_crit_desc(int idx, const gps_repo_t *r, char *buf, size_t cap)
{
    const gps_pvt_t *p = &r->pvt;
    bool ok = (r->crit & (1u << idx)) != 0;

    switch (idx) {
    case 0: /* CRIT_FIX_TYPE */
        snprintf(buf, cap,
                 "Fix geometrico 2D/3D/GPS+DR (NAV-SOL=%u [%s], NAV-STATUS=%u [%s])",
                 p->gpsFix_sol, gps_fix_name(p->gpsFix_sol),
                 p->gpsFix_st,  gps_fix_name(p->gpsFix_st));
        break;

    case 1: /* CRIT_FIX_OK */
        snprintf(buf, cap,
                 "Bandera gpsFixOk == 1 (SOL.flags=0x%02X [b0=%u], STATUS.flags=0x%02X [b0=%u])",
                 p->flags_sol, (unsigned)(p->flags_sol & 1u),
                 p->flags_st,  (unsigned)(p->flags_st & 1u));
        break;

    case 2: /* CRIT_GGA_Q */
        snprintf(buf, cap,
                 "NMEA GPGGA indicador FS >= 1 (actual FS=%d, SV=%d, recibida=%s)",
                 r->gga.valid ? r->gga.d.quality : 0,
                 r->gga.valid ? r->gga.d.numSV : 0,
                 r->gga.valid ? "SI" : "NO");
        break;

    case 3: /* CRIT_RMC_A */
        snprintf(buf, cap,
                 "NMEA GPRMC estado == 'A' Activo (actual='%c', modo='%c')",
                 (r->rmc.valid && r->rmc.d.status) ? r->rmc.d.status : 'V',
                 (r->rmc.valid && r->rmc.d.mode)   ? r->rmc.d.mode   : 'N');
        break;

    case 4: /* CRIT_UTC_VALID */
        snprintf(buf, cap,
                 "UBX-NAV-TIMEUTC bits validTOW+validWKN+validUTC == 0x07 (actual=0x%02X)",
                 (unsigned)(p->utc_valid & 0x07u));
        break;

    case 5: /* CRIT_TACC */
        snprintf(buf, cap,
                 "Incertidumbre temporal UTC tAcc < %u ns (actual=%lu ns)",
                 (unsigned)TRUST_MAX_TACC_NS,
                 (unsigned long)p->utc_tAcc);
        break;

    case 6: { /* CRIT_NUMSV */
        uint8_t req = (p->gpsFix_sol == 2) ? TRUST_MIN_SV_2D : TRUST_MIN_SV_3D;
        snprintf(buf, cap,
                 "Satelites en solucion numSV >= %u (actual=%u de %u, efem=%u, C/N0>=27=%u)",
                 req, p->numSV, req, r->sv_eph, r->sv_strong);
        break;
    }

    case 7: { /* CRIT_PDOP */
        uint16_t pd = (p->pDOP > 0) ? p->pDOP : p->pDOP_sol;
        snprintf(buf, cap,
                 "Dilucion de precision PDOP < %.1f (actual=%.2f, HDOP=%.2f, VDOP=%.2f)",
                 (double)TRUST_MAX_PDOP_X100 / 100.0,
                 (double)pd / 100.0,
                 (double)p->hDOP / 100.0,
                 (double)p->vDOP / 100.0);
        break;
    }

    case 8: /* CRIT_HACC */
        snprintf(buf, cap,
                 "Error horizontal estimado hAcc < %.1f m (actual=%.2f m, vAcc=%.2f m)",
                 (double)TRUST_MAX_HACC_MM / 1000.0,
                 (double)p->hAcc / 1000.0,
                 (double)p->vAcc / 1000.0);
        break;

    default:
        snprintf(buf, cap, "Criterio desconocido");
        break;
    }
    return ok;
}
