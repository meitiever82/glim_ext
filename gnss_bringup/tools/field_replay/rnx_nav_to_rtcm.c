/* rnx_nav_to_rtcm.c —— RINEX 星历(nav)文件转 RTCM3 星历电文,用于 Task 2
 * 现场回放"补星历"通道(方案 B,见 context.md 设计决定 2):GPS -> 1019、
 * GLONASS -> 1020、BDS -> 1042、Galileo -> 1046。
 *
 * 只依赖 RTKLIB-EX 2.5.1(`/usr/local/lib/librtklib.so`);不参与
 * gnss_bringup 的 CMake 构建,由 `run_field_integration.sh` 在运行时编译
 * (Task 3),编译方式与 `test/data/rnx2rtcm.c`(round2-hardening 计划
 * Task 6 Step 1)一致。
 *
 * usage: rnx_nav_to_rtcm <rover.nav> <out.rtcm3>
 *
 * 编译(-D 宏必须与 /usr/local/lib/librtklib.so 构建时一致,否则 obs_t/nav_t
 * 结构体布局对不上;命令抄自 round2-hardening 计划 Task 6 Step 1):
 *   RT=/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1
 *   gcc -O2 -DDLL -DENACMP -DENAGAL -DENAGLO -DENAIRN -DENAQZS -DNEXOBS=3 \
 *     -DNFREQ=3 -DTRACE -I$RT/src -o rnx_nav_to_rtcm rnx_nav_to_rtcm.c \
 *     -L/usr/local/lib -lrtklib -lm -lpthread
 */
#include <stdio.h>
#include <string.h>
#include "rtklib.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <rover.nav> <out.rtcm3>\n", argv[0]);
        return 1;
    }

    /* readrnx 对纯 nav 文件不会写 obs(obs.n 保持 0),这里给个哑元占位,
     * 与 test/data/rnx2rtcm.c 读 nav 文件时的用法一致。 */
    static obs_t dummy_obs;
    static nav_t nav;
    static rtcm_t rtcm;

    if (readrnx(argv[1], 1, "", &dummy_obs, &nav, NULL) < 0) {
        fprintf(stderr, "nav read failed: %s\n", argv[1]);
        return 1;
    }
    if (!init_rtcm(&rtcm)) {
        fprintf(stderr, "init_rtcm failed\n");
        return 1;
    }

    FILE *fp = fopen(argv[2], "wb");
    if (!fp) {
        fprintf(stderr, "cannot open output: %s\n", argv[2]);
        free_rtcm(&rtcm);
        return 1;
    }

    int n1019 = 0, n1020 = 0, n1042 = 0, n1046 = 0, n_skipped = 0;

    /* GPS(1019)/BDS(1042)/Galileo I/NAV(1046):对 nav.eph 里读到的每条星历逐一
     * 编码(不是每颗卫星只发一条最新的,是每条历元记录都发一条 RTCM 电文)。
     * rtcm.nav.eph 是按卫星号(ephsat-1)索引的固定数组(init_rtcm 分配了
     * MAXSAT乘2项),各卫星系统的 satno() 编号区间互不重叠(见 rtklib.h 里
     * MINPRN 与 NSAT 系列宏),同一时刻放一条互不覆盖。encode_type1019、
     * encode_type1042、encode_type1046(见 rtcm3e.c)都是取
     * rtcm->nav.eph + rtcm->ephsat - 1 处的星历,已核实。 */
    for (int k = 0; k < nav.n; k++) {
        eph_t *eph = &nav.eph[k];
        int prn = 0;
        int sys = satsys(eph->sat, &prn);
        int type;
        if (sys == SYS_GPS) {
            type = 1019;
        } else if (sys == SYS_CMP) {
            type = 1042;
        } else if (sys == SYS_GAL) {
            type = 1046;
        } else {
            continue; /* QZSS/IRNSS 等本任务不需要,rover.nav 里也没有 */
        }

        rtcm.nav.eph[eph->sat - 1] = *eph;
        rtcm.ephsat = eph->sat;
        rtcm.ephset = 0;
        if (gen_rtcm3(&rtcm, type, 0, 0)) {
            fwrite(rtcm.buff, rtcm.nbyte, 1, fp);
            if (type == 1019) n1019++;
            else if (type == 1042) n1042++;
            else n1046++;
        } else {
            n_skipped++;
        }
    }

    /* GLONASS(1020):encode_type1020(rtcm3e.c)取的是 `rtcm->nav.geph + prn - 1`
     * (prn 是 GLONASS 槽位号,来自 satsys(rtcm->ephsat,&prn)),不是 ephsat-1;
     * init_rtcm 给 rtcm.nav.geph 分配了 MAXPRNGLO 项,已核实。 */
    for (int k = 0; k < nav.ng; k++) {
        geph_t *geph = &nav.geph[k];
        int prn = 0;
        if (satsys(geph->sat, &prn) != SYS_GLO) continue;

        rtcm.nav.geph[prn - 1] = *geph;
        rtcm.ephsat = geph->sat;
        rtcm.ephset = 0;
        if (gen_rtcm3(&rtcm, 1020, 0, 0)) {
            fwrite(rtcm.buff, rtcm.nbyte, 1, fp);
            n1020++;
        } else {
            n_skipped++;
        }
    }

    fclose(fp);
    free_rtcm(&rtcm);

    printf("%s: 1019=%d 1020=%d 1042=%d 1046=%d skipped=%d\n", argv[2],
           n1019, n1020, n1042, n1046, n_skipped);
    return 0;
}
