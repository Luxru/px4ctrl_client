#include "client.h"
#include "datas.h"
#include "types.h"

#include <cstdint>
#include <imgui.h>
#include <vector>
#include <zmq_addon.hpp>

namespace  px4ctrl{
    namespace ui{
        Px4Client::Px4Client(
                zmq::context_t& ctx,
                const ZmqParas& paras
            ):paras(paras),
            client_pub(ctx,zmq::socket_type::pub),
            server_sub(ctx,zmq::socket_type::sub),
            log_sub(ctx,zmq::socket_type::sub)
            {
                try {
                    client_pub.connect(paras.xsub_url);
                    server_sub.connect(paras.xpub_url);
                    log_sub.connect(paras.xpub_url);
                    server_sub.set(zmq::sockopt::subscribe, paras.server_topic);
                    log_sub.set(zmq::sockopt::subscribe, paras.log_topic);
                } catch (const zmq::error_t &e) {
                    spdlog::error("unable bind server/pub, err:{}", e.what());
                    throw e;
                }
                recv_server_thread = std::thread(&Px4Client::recv_server, this);
                recv_log_thread = std::thread(&Px4Client::recv_log, this);
                return;
            }

        void Px4Client::pub_client(const ClientPayload& payload){
            if (!ok) {
                spdlog::error("zmq socket is shutdown");
                return;
            }
            client_pub.send(zmq::buffer(paras.client_topic), zmq::send_flags::sndmore);
            client_pub.send(zmq::buffer(&payload, sizeof(ClientPayload)));
            return;
        }

        void Px4Client::recv_server(){
            while(ok){
                    try {
                        zmq::message_t topic,data;
                        zmq::recv_result_t result = server_sub.recv(topic);
                        result = server_sub.recv(data);
                        ServerPayload payload;
                        unpack(data,payload);
                        server_data.post(payload);
                    } catch (const zmq::error_t &e) {
                        if(e.num() == ETERM){
                            spdlog::info("server recv thread exited");
                            if(ok)
                            {
                                {
                                    std::lock_guard<std::mutex> lock(mtx);
                                    ok = false;
                                }
                                server_sub.close();
                                log_sub.close();
                                client_pub.close();
                            }
                        }
                        spdlog::error("server recv error:{}",e.what());
                    }
                }
            return;
        }

        void Px4Client::recv_log(){
            while(ok){
                    try {
                        zmq::message_t topic,data;
                        zmq::recv_result_t result = log_sub.recv(topic);
                        result = log_sub.recv(data);
                        // spdlog::info("log recv, size:{}",data.size());
                        log_data.post(std::string(static_cast<char*>(data.data()),data.size()));
                    } catch (const zmq::error_t &e) {
                        if(e.num() == ETERM){
                            spdlog::info("log recv thread exited");
                            if(ok)
                            {
                                {
                                    std::lock_guard<std::mutex> lock(mtx);
                                    ok = false;
                                }
                                server_sub.close();
                                log_sub.close();
                                client_pub.close();
                            }
                        }
                        spdlog::error("log recv error:{}",e.what());
                    }
            }
            return;
        }

        Px4Client::~Px4Client(){
            {
                std::lock_guard<std::mutex> lock(mtx);
                ok = false;
            }
            if(recv_server_thread.joinable()){
                recv_server_thread.join();
            }
            if(recv_log_thread.joinable()){
                recv_log_thread.join();
            }
            spdlog::info("zmq client exit");
            return;
        }

        ImguiClient::ImguiClient(Px4Client& px4_client):px4_client(px4_client){
            log_observer = px4_client.log_data.observe(
                [&](const std::string& data){
                    log_data.push_back(data);
                }
            );

            server_observer = px4_client.server_data.observe(
                [&](const ServerPayload& data){
                    server_data_map[data.id] = data;
                }
            );

            command_vec = std::vector<ClientCommand>({
                // ClientCommand::HEARTBEAT,
                ClientCommand::ARM,
                ClientCommand::ENTER_OFFBOARD,
                ClientCommand::EXIT_OFFBOARD,
                ClientCommand::TAKEOFF,
                ClientCommand::LAND,
                ClientCommand::FORCE_HOVER,
                ClientCommand::ALLOW_CMD_CTRL,
                ClientCommand::FORCE_DISARM,
                // ClientCommand::CHANGE_HOVER_POS,
            });

            return;
        }

        void ImguiClient::render_window(){
            ImGuiIO& io = ImGui::GetIO(); (void)io;
            ImGui::Begin("UISC Px4 Client", NULL , ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoBackground);                          // Create a window called "Hello, world!" and append into it.
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            for(const auto& elem:server_data_map){
                const uint8_t id = elem.first;
                const ServerPayload& drone = elem.second;
                //Window
                ImGui::PushID(fmt::format("Drone{}",id).c_str());
                if(ImGui::BeginChild("DroneContainer",ImVec2(0,0),true)){
                    ImGui::Text("Drone ID: %d",id);
                    // Two columns
                    ImGui::Columns(2);
                    ImGui::Text("FSM Status: %s %s %s",Px4CtrlStateName[drone.fsm_state[0]],Px4CtrlStateName[drone.fsm_state[1]],Px4CtrlStateName[drone.fsm_state[2]]);
                    ImGui::Text("pos: %.2f %.2f %.2f",drone.pos[0],drone.pos[1],drone.pos[2]);
                    ImGui::Text("vel: %.2f %.2f %.2f",drone.vel[0],drone.vel[1],drone.vel[2]);
                    ImGui::Text("omega: %.2f %.2f %.2f",drone.omega[0],drone.omega[1],drone.omega[2]);
                    ImGui::Text("quat: %.2f %.2f %.2f %.2f",drone.quat[0],drone.quat[1],drone.quat[2],drone.quat[3]);
                    ImGui::Text("battery: %.2f",drone.battery_voltage);
                    ImGui::Text("L2 Hover pos: %.2f %.2f %.2f",drone.hover_pos[0],drone.hover_pos[1],drone.hover_pos[2]);
                    ImGui::Text("L2 Hover quat: %.2f %.2f %.2f %.2f",drone.hover_quat[0],drone.hover_quat[1],drone.hover_quat[2],drone.hover_quat[3]);
                    ImGui::Text("Last update: %.2f s",timePassedSeconds(from_uint64(drone.timestamp)));
                    
                    ImGui::NextColumn();
                    ImGui::PushID(id);
                    for(const auto &cmd : command_vec){
                        if(ImGui::Button(CommandStr[(int)cmd])){
                            spdlog::info("Recv Command");
                            ClientPayload payload;
                            payload.command = cmd;
                            payload.timestamp = to_uint64(clock::now());
                            px4_client.pub_client(payload);
                        }
                    }
                    ImGui::PopID();
                    ImGui::Columns(1);
                    //Scroll to bottom log
                    if(ImGui::BeginChild("LogContainer",ImVec2(0,0),true)){
                        for(const auto& log:log_data){
                            ImGui::Text("%s",log.c_str());
                        }
                        ImGui::SetScrollHereY(1.0f);
                        ImGui::EndChild();
                    }
                    ImGui::EndChild();
                }
                ImGui::PopID();
            }
            if(server_data_map.empty()){
                ImGui::Text("No drone");
            }
            ImGui::End();
            return;
        }
        
    }//namespace ui
}//namespace px4ctrl