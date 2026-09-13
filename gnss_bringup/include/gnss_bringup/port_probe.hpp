#pragma once

namespace gnss_bringup {

// 探测 127.0.0.1:port 是否已有人监听。用于启动时发现残留的 rtkrcv 孤儿,
// 或误开的第二个实例——两者都会让本节点连上别人的解流,把陈旧解当新鲜数据发出去。
bool is_local_port_listening(int port);

}  // namespace gnss_bringup
