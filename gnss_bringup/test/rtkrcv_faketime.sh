#!/usr/bin/env bash
# 测试专用:在 libfaketime 下 exec 真实 rtkrcv。
# - 用 exec,pid 不变,节点发的 SIGTERM 直接到 rtkrcv(faketime 命令会 fork,不能用)
# - 用多线程版 libfaketimeMT:rtkrcv 是多线程的
# - 回放数据是 2005-04-02 的 RTCM3,rtkrcv 用系统时间补 RTCM 周数,时钟必须在同一周
exec env LD_PRELOAD="${GNSS_TEST_FAKETIME_LIB:?}" FAKETIME="@2005-04-02 03:00:00" TZ=UTC rtkrcv "$@"
