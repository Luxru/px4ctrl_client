#pragma once
#include "datas.h"
#include "types.h"

#include <array>
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

    inline std::array<double, 4> q_inv(const std::array<double, 4>& q){
        // q = [w, x, y, z]
        return {q[0], -q[1], -q[2], -q[3]};
    }
    
    inline std::array<double, 4> q_mul(const std::array<double, 4>& q1, const std::array<double, 4>& q2){
        //return q1*q2
        // q = [w, x, y, z]
        return {
            q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
        };
    }
    
    inline std::array<double, 3> q_rot(const std::array<double, 4>& q, const std::array<double, 3>& v){
        //return q*v*q_inv
        // q = [w, x, y, z]
        std::array<double, 4> q_v = {0, v[0], v[1], v[2]};
        std::array<double, 4> q_inv_v = q_mul(q, q_mul(q_v, q_inv(q)));
        return {q_inv_v[1], q_inv_v[2], q_inv_v[3]};
    }

    inline double to_yaw(const std::array<double, 4>& q){
        // q = [w, x, y, z]
        // atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
        return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    }

    inline std::array<double, 4> from_yaw(double yaw){
        // The quaternion representation of rotation is a variation on axis and angle. So if you rotate by r radians around axis x, y, z, then your quaternion q is:
        // q = [cos(r/2), sin(r/2)x, sin(r/2)y, sin(r/2)z]
        return {std::cos(yaw/2), 0, 0, std::sin(yaw/2)};
    }
}
}