#include "client.h"
#include "datas.h"
#include "types.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
                ClientCommand::RESTART_FCU,
                // ClientCommand::CHANGE_HOVER_POS,
            });

            return;
        }

        void ImguiClient::render_window(){
            ImGuiIO& io = ImGui::GetIO(); (void)io;
            ImGui::Begin("UISC Px4 Client", NULL , ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoBackground);                          // Create a window called "Hello, world!" and append into it.
            //Same line
            ImGui::Text("UISC Px4 Client");
            ImGui::SameLine();
            ImGui::Text("v0.1");
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::BeginItemTooltip())
            {
                ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
                ImGui::TextUnformatted("Press W/S/A/D to move drone in XY plane\nPress R/F to move drone in Z axis\nPress Q/E to rotate drone\nPress Space to force hover\nPress J to force disarm\nPress L to land\nPress C to change Control Frame");
                ImGui::PopTextWrapPos();
                ImGui::EndTooltip();
            }
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::SameLine();
            if(ctrl_in_world){
                // control in world frame
                ImGui::TextColored(ImVec4(1,0,0,1), "Control in World Frame");
            }else{
                // control in body frame
                ImGui::TextColored(ImVec4(0,1,0,1), "Control in Body Frame");
            }
            ImGui::SameLine();
            if(ImGui::Button("Change Control Frame")){
                ctrl_in_world = !ctrl_in_world;
            }
            if(ImGui::IsKeyPressed(ImGuiKey_C)){
                ctrl_in_world = !ctrl_in_world;
            }

            const double delta_time_frame = io.DeltaTime;

            //velocity in body frame
            const double velocity_xy = 1; // TODO change velocity in config
            const double velocity_z =  0.2;
            const double velocity_yaw = 2;// ~120 degree/s
            for(const auto& elem:server_data_map){
                const uint8_t id = elem.first;
                const ServerPayload& drone = elem.second;
                std::array<double,3> hover_pos = {drone.hover_pos[0],drone.hover_pos[1],drone.hover_pos[2]};
                std::array<double, 4> hover_quat = {drone.hover_quat[0],drone.hover_quat[1],drone.hover_quat[2],drone.hover_quat[3]};
                //Window
                ImGui::PushID(std::format("Drone{}",id).c_str());
                if(ImGui::BeginChild("DroneContainer",ImVec2(0,0),true)){
                    ImGui::Text("Drone ID: %d",id);
                    // Red Text
                    if(ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)){
                        // Green Text
                        ImGui::TextColored(ImVec4(0,1,0,1), "Active");
                        {
                            double yaw = to_yaw(hover_quat);
                            //first, get veclocity in world frame
                            std::array<double,3> vel_body_x = {velocity_xy,0,0};//body frame
                            std::array<double,3> vel_body_y = {0,velocity_xy,0};//body frame
                            std::array<double, 4> quat = {drone.quat[0],drone.quat[1],drone.quat[2],drone.quat[3]};
                            std::array<double,3> vel_world_x,vel_world_y;
                            if(ctrl_in_world){
                                //velocity in world frame
                                vel_world_x = vel_body_x;
                                vel_world_y = vel_body_y;
                            }else{
                                //velocity in body frame
                                vel_world_x = q_rot(quat, vel_body_x);
                                vel_world_y = q_rot(quat, vel_body_y);
                            }
                            // TODO change velocity in config
                            bool hover_pos_changed = false;
                            if(ImGui::IsKeyDown(ImGuiKey_W)){
                                hover_pos[0] += vel_world_x[0]*delta_time_frame;
                                hover_pos[1] += vel_world_x[1]*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_S)){
                                hover_pos[0] -= vel_world_x[0]*delta_time_frame;
                                hover_pos[1] -= vel_world_x[1]*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_D)){
                                hover_pos[0] -= vel_world_y[0]*delta_time_frame;
                                hover_pos[1] -= vel_world_y[1]*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_A)){
                                hover_pos[0] += vel_world_y[0]*delta_time_frame;
                                hover_pos[1] += vel_world_y[1]*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_Q)){
                                // yaw
                                yaw += velocity_yaw*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_E)){
                                // yaw
                                yaw -= velocity_yaw*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(ImGui::IsKeyDown(ImGuiKey_R)){
                                // z
                                hover_pos[2] += velocity_z*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if (ImGui::IsKeyDown(ImGuiKey_F)){
                                // z
                                hover_pos[2] -= velocity_z*delta_time_frame;
                                hover_pos_changed = true;
                            }
                            if(hover_pos_changed){
                                //update hover pos
                                ClientPayload payload;
                                payload.command = ClientCommand::CHANGE_HOVER_POS;
                                payload.timestamp = to_uint64(clock::now());
                                payload.id = id;
                                double data[7];
                                data[0] = hover_pos[0];
                                data[1] = hover_pos[1];
                                data[2] = hover_pos[2];
                                auto new_quat = from_yaw(yaw);
                                data[3] = new_quat[0];
                                data[4] = new_quat[1];
                                data[5] = new_quat[2];
                                data[6] = new_quat[3];
                                std::memcpy(payload.data,data,sizeof(data));
                                px4_client.pub_client(payload);
                            }

                            if (ImGui::IsKeyPressed(ImGuiKey_Space)){
                                spdlog::debug("Space key pressed");//force hover
                                ClientPayload payload;
                                payload.command = ClientCommand::FORCE_HOVER;
                                payload.timestamp = to_uint64(clock::now());
                                px4_client.pub_client(payload);
                            }

                            if (ImGui::IsKeyPressed(ImGuiKey_J)){
                                spdlog::debug("J key pressed");//Force Disarm
                                ClientPayload payload;
                                payload.command = ClientCommand::FORCE_DISARM;
                                payload.timestamp = to_uint64(clock::now());
                                px4_client.pub_client(payload);
                            }

                            if (ImGui::IsKeyPressed(ImGuiKey_L)){
                                spdlog::debug("L key pressed");//Land
                                ClientPayload payload;
                                payload.command = ClientCommand::LAND;
                                payload.timestamp = to_uint64(clock::now());
                                px4_client.pub_client(payload);
                            }
                        }
                    }else{
                        // Red Text
                        ImGui::TextColored(ImVec4(1,0,0,1), "Inactive");
                    }
                    // Two columns
                    if (ImGui::BeginTable("MainCTN", 2, ImGuiTableFlags_Resizable|ImGuiTableFlags_BordersV|ImGuiTableFlags_SizingFixedFit)){
                        ImGui::TableSetupColumn("stat",ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("cmd");
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        {
                            ImGui::BeginGroup();
                            ImGui::Text("L0: ");
                            ImGui::SameLine();
                            for(int i=0;i<L0_L1;++i){
                                if(drone.fsm_state[0]==i){
                                    // active color
                                    ImGui::TextColored(ImVec4(0,1,0,1),"%s",Px4CtrlStateName[i]);
                                }else{
                                    // not active color
                                    ImGui::TextDisabled("%s",Px4CtrlStateName[i]);
                                }
                                ImGui::SameLine();
                            }
                            ImGui::EndGroup();
                            ImGui::BeginGroup();
                            ImGui::Text("L1: ");
                            ImGui::SameLine();
                            for(int i=L0_L1+1;i<L1_L2;++i){
                                if(drone.fsm_state[1]==i){
                                    // active color
                                    ImGui::TextColored(ImVec4(0,1,0,1),"%s",Px4CtrlStateName[i]);
                                }else{
                                    // not active color
                                    ImGui::TextDisabled("%s",Px4CtrlStateName[i]);
                                }
                                ImGui::SameLine();
                            }
                            ImGui::EndGroup();
                            ImGui::BeginGroup();
                            ImGui::Text("L2: ");
                            ImGui::SameLine();
                            for(int i=L1_L2+1;i<END;++i){
                                if(drone.fsm_state[2]==i){
                                    // active color
                                    ImGui::TextColored(ImVec4(0,1,0,1),"%s",Px4CtrlStateName[i]);
                                }else{
                                    // not active color
                                    ImGui::TextDisabled("%s",Px4CtrlStateName[i]);
                                }
                                ImGui::SameLine();
                            }
                            ImGui::EndGroup();
                            
                            ImGui::Text("pos: %.2f %.2f %.2f",drone.pos[0],drone.pos[1],drone.pos[2]);
                            ImGui::Text("vel: %.2f %.2f %.2f",drone.vel[0],drone.vel[1],drone.vel[2]);
                            ImGui::Text("omega: %.2f %.2f %.2f",drone.omega[0],drone.omega[1],drone.omega[2]);
                            ImGui::Text("quat: %.2f %.2f %.2f %.2f",drone.quat[0],drone.quat[1],drone.quat[2],drone.quat[3]);
                            ImGui::Text("battery: %.2f",drone.battery_voltage);
                            ImGui::Text("L2 Hover pos: %.2f %.2f %.2f",drone.hover_pos[0],drone.hover_pos[1],drone.hover_pos[2]);
                            ImGui::Text("L2 Hover quat: %.2f %.2f %.2f %.2f",drone.hover_quat[0],drone.hover_quat[1],drone.hover_quat[2],drone.hover_quat[3]);
                            ImGui::Text("odom hz: %.2f, cmdctrl hz: %.2f",drone.odom_hz,drone.cmdctrl_hz);
                            ImGui::Text("Last update: %.2f s",timePassedSeconds(from_uint64(drone.timestamp)));
                        }
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Indent(10.0f); // Add left padding of 10 pixels
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
                        // Add popup
                        static char bufx[32] = "";
                        static char bufy[32] = ""; 
                        static char bufz[32] = ""; 
                        static char bufyaw[32] = ""; 
                        if (ImGui::Button("SET_HOVER_POSITION")){
                            ImGui::OpenPopup("SET_HOVER");
                            std::sprintf(bufx,"%.2f",drone.pos[0]);
                            std::sprintf(bufy,"%.2f",drone.pos[1]);
                            std::sprintf(bufz,"%.2f",drone.pos[2]);
                            std::sprintf(bufyaw,"%.2f",to_yaw(hover_quat));
                        }
                        // Always center this window when appearing
                        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
                        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

                        if (ImGui::BeginPopupModal("SET_HOVER", NULL, ImGuiWindowFlags_AlwaysAutoResize))
                        {
                            ImGui::Text("The set position must be less than 1 meter away from the current position.\n Current position: %.2f %.2f %.2f",drone.pos[0],drone.pos[1],drone.pos[2]);
                            ImGui::Separator();
                            ImGui::InputText("X", bufx, 32, ImGuiInputTextFlags_CharsDecimal);
                            ImGui::InputText("Y", bufy, 32, ImGuiInputTextFlags_CharsDecimal);
                            ImGui::InputText("Z", bufz, 32, ImGuiInputTextFlags_CharsDecimal);
                            ImGui::InputText("Yaw",bufyaw,32,ImGuiInputTextFlags_CharsDecimal);
                            // TODO check
                            double x = std::atof(bufx);
                            double y = std::atof(bufy);
                            double z = std::atof(bufz);
                            double yaw = std::atof(bufyaw);
                            bool number_valid = !(std::isnan(drone.pos[0])||std::isnan(drone.pos[1])||std::isnan(drone.pos[2])||
                            std::isnan(x)||std::isnan(y)||std::isnan(z)||std::isnan(yaw));
                            bool valid = number_valid;
                            if(number_valid){
                                valid = std::sqrt(
                                    std::pow(x-drone.pos[0],2)+
                                    std::pow(y-drone.pos[1],2)+
                                    std::pow(z-drone.pos[2],2)
                                )<=1.5f;
                            }
                            if(!valid){
                                ImGui::TextColored(ImVec4(1,0,0,1),"Invalid Input x,y,z: %2f, %2f, %2f. Dist: %2f",x,y,z,std::sqrt(
                                    std::pow(x-drone.pos[0],2)+
                                    std::pow(y-drone.pos[1],2)+
                                    std::pow(z-drone.pos[2],2)
                                ));
                            }
                            ImGui::BeginDisabled(!valid);
                            if (ImGui::Button("OK", ImVec2(120, 0))) {
                                //update hover pos
                                ClientPayload payload;
                                payload.command = ClientCommand::CHANGE_HOVER_POS;
                                payload.timestamp = to_uint64(clock::now());
                                payload.id = id;
                                double data[7];
                                data[0] = x;
                                data[1] = y;
                                data[2] = z;
                                auto new_quat = from_yaw(yaw);
                                data[3] = new_quat[0];
                                data[4] = new_quat[1];
                                data[5] = new_quat[2];
                                data[6] = new_quat[3];
                                std::memcpy(payload.data,data,sizeof(data));
                                px4_client.pub_client(payload);
                                // check
                                ImGui::CloseCurrentPopup(); 
                            }
                            ImGui::EndDisabled();
                            ImGui::SetItemDefaultFocus();
                            ImGui::SameLine();
                            if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
                            ImGui::EndPopup();
                        }
                        ImGui::Unindent(10.0f); // Reset indentation
                        ImGui::EndTable();
                    }
                }
                //Log panel
                {
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