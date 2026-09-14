/* RINEX obs+nav -> RTCM3:每个历元一条 1004,每 10 个历元补发 1019(GPS 星历)+ 1005(基准站坐标)。
 * 只用于生成 gnss_bringup 的回放测试数据,编译方法见同目录 README.md。 */
#include <stdio.h>
#include <string.h>
#include "rtklib.h"

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s obs nav out.rtcm3\n", argv[0]); return 1; }
    static obs_t obs; static nav_t nav; static sta_t sta; static obs_t dum; static rtcm_t rtcm;
    if (readrnx(argv[1], 1, "", &obs, &nav, &sta) < 0) { fprintf(stderr, "obs read failed\n"); return 1; }
    if (readrnx(argv[2], 1, "", &dum, &nav, NULL) < 0) { fprintf(stderr, "nav read failed\n"); return 1; }
    sortobs(&obs);
    if (!init_rtcm(&rtcm)) return 1;
    rtcm.sta = sta; rtcm.staid = 1;
    FILE *fp = fopen(argv[3], "wb"); if (!fp) return 1;
    int ep = 0, n1004 = 0, n1019 = 0, n1005 = 0;
    for (int i = 0; i < obs.n; ep++) {
        int m = 0; gtime_t t = obs.data[i].time;
        while (i + m < obs.n && fabs(timediff(obs.data[i + m].time, t)) < 1e-3) m++;
        if (ep % 10 == 0) {
            for (int s = 1; s <= MAXSAT; s++) {
                if (satsys(s, NULL) != SYS_GPS) continue;
                int best = -1; double bd = 1e9;
                for (int k = 0; k < nav.n; k++) {
                    if (nav.eph[k].sat != s) continue;
                    double d = fabs(timediff(nav.eph[k].toe, t));
                    if (d < bd) { bd = d; best = k; }
                }
                if (best < 0 || bd > 7200.0) continue;
                rtcm.nav.eph[s - 1] = nav.eph[best]; rtcm.ephsat = s; rtcm.ephset = 0;
                if (gen_rtcm3(&rtcm, 1019, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1019++; }
            }
            if (gen_rtcm3(&rtcm, 1005, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1005++; }
        }
        rtcm.time = t; rtcm.obs.n = m;
        memcpy(rtcm.obs.data, obs.data + i, sizeof(obsd_t) * m);
        if (gen_rtcm3(&rtcm, 1004, 0, 0)) { fwrite(rtcm.buff, rtcm.nbyte, 1, fp); n1004++; }
        i += m;
    }
    fclose(fp);
    printf("%s: epochs=%d 1004=%d 1019=%d 1005=%d\n", argv[3], ep, n1004, n1019, n1005);
    return 0;
}
