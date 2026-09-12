#!/usr/bin/env bash
# 假 rtkrcv:用于在没有 RTKLIB 的机器上测试进程监管。
#   fake_rtkrcv.sh die              —— 立即退出(模拟 conf 错误导致的崩溃循环)
#   fake_rtkrcv.sh live             —— 长活直到被杀,一收到 SIGTERM 立刻退出
#   fake_rtkrcv.sh live-count <f>   —— 长活;每收到一次 SIGTERM/SIGINT 就把
#     累计次数写进文件 <f>,但故意不在第一次就退出(最多撑 2s,或者收满
#     10 次才主动退出)——用来验证 stop() 真的只发了个位数的信号,而不是
#     像修复前那样每 20ms 一次、撑满 5s 打出上百次(那样会让一个真的想在
#     退出前 flush 数据的目标程序,信号处理函数不断被打断,永远做不完那次
#     收尾)。live 模式一碰就倒,没法拿来数到底挨了几下,所以另开一种模式。
# 其余参数(-s -nc -r 2 -o <conf>)一律忽略,只用于验证传参不会导致启动失败。
mode="die"
countfile=""
prev=""
for a in "$@"; do
  case "$a" in
    die|live|live-count) mode="$a" ;;
    *) if [[ "$prev" == "live-count" ]]; then countfile="$a"; fi ;;
  esac
  prev="$a"
done

if [[ "$mode" == "live-count" ]]; then
  count=0
  on_signal() {
    count=$((count + 1))
    echo "$count" > "$countfile"
    if [[ "$count" -ge 10 ]]; then
      exit 0
    fi
  }
  trap on_signal TERM INT
  for _ in $(seq 1 40); do
    sleep 0.05
  done
  exit 0
fi

if [[ "$mode" == "live" ]]; then
  trap 'exit 0' TERM INT
  while true; do sleep 0.1; done
fi
exit 1
