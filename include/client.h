#pragma once
#include "datas.h"
#include "types.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace px4ctrl{
namespace ui{
    class Px4Client{
        public:
            Px4Client(
                zmq::context_t& context,
                const ZmqParas& paras
            );
            ~Px4Client();
            Px4Data<ServerPayload> server_data;
            Px4Data<std::string> log_data;
            void pub_client(const ClientPayload& payload);
        private:
            const ZmqParas& paras;
            zmq::socket_t client_pub,server_sub, log_sub;

            std::mutex mtx;
            bool ok = true;
            std::thread recv_server_thread, recv_log_thread;
            void recv_server();
            void recv_log();
    };

    class ImguiClient{
        public:
            ImguiClient(Px4Client& px4_client);
            void render_window();
        private:
            std::vector<ClientCommand> command_vec;
            std::map<uint8_t, ServerPayload> server_data_map;
            Px4DataObserver log_observer,server_observer;
            std::vector<std::string> log_data;
            Px4Client& px4_client;
    };
}
}