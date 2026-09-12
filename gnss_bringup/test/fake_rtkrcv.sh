#!/usr/bin/env bash
# 假 rtkrcv:用于在没有 RTKLIB 的机器上测试进程监管。
#   fake_rtkrcv.sh die      —— 立即退出(模拟 conf 错误导致的崩溃循环)
#   fake_rtkrcv.sh live     —— 长活直到被杀
# 其余参数(-s -nc -r 2 -o <conf>)一律忽略,只用于验证传参不会导致启动失败。
mode="die"
for a in "$@"; do case "$a" in die|live) mode="$a";; esac; done
if [[ "$mode" == "live" ]]; then
  trap 'exit 0' TERM INT
  while true; do sleep 0.1; done
fi
exit 1
