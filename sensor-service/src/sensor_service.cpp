/*
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * Not a Contribution.
 *
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *            http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sns_direct_channel.pb.h"
#include "sns_suid.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_direct_channel.h"
#include "AEEStdErr.h"
#include "rpcmem.h"
#include "remote.h"
#include "sns_direct_channel_buffer.h"
#include "sensors_timeutil.h"
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdlib.h>
#include <json/json.h>
#include <bsd/string.h>
#include <mutex>
#include <thread>

#define DIRECT_CHANNEL_SHARED_MEMORY_SIZE (1000*104)
#define CONFIG_FILE_PATH ("/etc/sensors_info.conf")
#define CONFIG_FILE_DEFAULT_PATH ("/etc/sensors_info_conf.default")
#define ANDROID_MUX_CHANNEL 0
#define GENERIC_CHANNEL 1
#define MAX_PATH_LEN 255
#define SENSOR_TYPE      "sensor_type"
#define CALIBRATED       "calibrated"
#define RESAMPLED        "resampled"
#define SAMPLE_RATE      "sample_rate"
#define CHANNEL_TYPE     "channel_type"
#define SensorsConfVer "1.0"
#define SENSOR_ACCEL "accel"
#define SENSOR_GYRO "gyro"
#define MAX_STRING_LEN 255

#define SOCKET_PATH "/dev/shm/server_socket"
#define MAX_SOCK_CLIENT 10
#define STOP 1
#define CURRENT_SUPPORT_SENSOR_NUM 2

bool is_handle_available = false;
remote_handle64 fastRPC_remote_handle = -1;
remote_handle64 remote_handle_fd = 0;
std::vector<int> sensors_fd;
volatile int num = 0;
int original_sample_rate = 240;
int adjusted_sample_rate = 240;
std::mutex mtx;
std::vector<sensors> sensors_info_vec_;
std::vector<suid_info> suid_info_list_;
std::vector<int> supported_sample_rate_;
std::vector<int> channel_handle_;
std::vector<char*> sensor_buff_ptr_;


int fastRPC_remote_handle_init()
{
    if (is_handle_available == true) {
        std::cout << "is_handle_available not available " << std::endl;
        return -1;
    }

    int nErr = AEE_SUCCESS;
    std::string uri = "file:///libsns_direct_channel_skel.so?sns_direct_channel_skel_handle_invoke&_modver=1.0";
    remote_handle64 handle_l;
    struct stat sb;

    /*check for slpi or adsp*/
    if (!stat("/sys/kernel/boot_slpi", &sb)) {
        uri += "&_dom=sdsp";
        if (remote_handle64_open(ITRANSPORT_PREFIX "createstaticpd:sensorspd&_dom=sdsp", &remote_handle_fd)) {
            std::cout << "failed to open remote handle for sensorspd - sdsp" << std::endl;
            return -1;
        }
    } else {
        uri += "&_dom=adsp";
        if (remote_handle64_open(ITRANSPORT_PREFIX "createstaticpd:sensorspd&_dom=adsp", &remote_handle_fd)) {
            std::cout << "failed to open remote handle for sensorspd - adsp" << std::endl;
            return -1;
        }
    }

    if (AEE_SUCCESS == (nErr = sns_direct_channel_open(uri.c_str(), &handle_l))) {
        fastRPC_remote_handle = handle_l;
        is_handle_available = true;
        return 0;
    } else {
        fastRPC_remote_handle = -1;
        std::cout << "sns_direct_channel_open failed" << std::endl;
        return -1;
    }
    return 0;
}

int create_rpc_memory(char* &sensor_buff_ptr) {
    sensor_buff_ptr = (char*)rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, DIRECT_CHANNEL_SHARED_MEMORY_SIZE);
    if (nullptr == sensor_buff_ptr) {
        return -1;
    }
    int fd = rpcmem_to_fd((void *)sensor_buff_ptr);
    return fd;
}

void delete_direct_channel(int channel_handle, char* &sensor_buff_ptr) {
    if (sns_direct_channel_delete(fastRPC_remote_handle, channel_handle) != 0) {
        std::cout << "delete_direct_channel fail " << std::endl;
    }
    rpcmem_free((void*)sensor_buff_ptr);
}

int create_direct_channel(bool is_generic_channel, char* &sensor_buff_ptr, bool save_fd = false, std::string sensor_type_name = "") {
    int fd = create_rpc_memory(sensor_buff_ptr);
    if (fd < 0) {
      std::cout << "create_rpc_memory fail " << std::endl;
    }

    if (true == save_fd) sensors_fd.emplace_back(fd);

    sns_direct_channel_create_msg create_msg;
    sns_direct_channel_create_msg_shared_buffer_config *shared_buffer_config = create_msg.mutable_buffer_config();
    if (nullptr == shared_buffer_config) return -1;

    shared_buffer_config->set_fd(fd);
    shared_buffer_config->set_size(DIRECT_CHANNEL_SHARED_MEMORY_SIZE);

    if (true == is_generic_channel)
        create_msg.set_channel_type(DIRECT_CHANNEL_TYPE_GENERIC_CHANNEL);
    else
        create_msg.set_channel_type(DIRECT_CHANNEL_TYPE_STRUCTURED_MUX_CHANNEL);

    create_msg.set_client_proc(SNS_STD_CLIENT_PROCESSOR_APSS);

    std::string pb_encoded_direct_channel_req_msg;
    create_msg.SerializeToString(&pb_encoded_direct_channel_req_msg);

    int channel_handle = 0;
    int ret = sns_direct_channel_create(fastRPC_remote_handle, (const unsigned char*)pb_encoded_direct_channel_req_msg.c_str(), pb_encoded_direct_channel_req_msg.size(), &channel_handle);
    if (0 != ret) {
        std::cout << "sensor_direct_channel sns_direct_channel_create failed " << std::endl;
    }

    return channel_handle;
}

void send_config_request(int channel_handle, sns_request_type req_type, sensors sensor, int adjusted_sample_rate = 240) {
    sns_direct_channel_config_msg config_msg;
    sns_direct_channel_set_client_req* req_msg = config_msg.mutable_set_client_req();
    if (nullptr == req_msg)
        return;

    if (req_type == SNS_GENERIC_SUID)
        req_msg->set_msg_id(SNS_SUID_MSGID_SNS_SUID_REQ);

    if (req_type == SNS_GENERIC_ATTRIBUTES)
        req_msg->set_msg_id(SNS_STD_MSGID_SNS_STD_ATTR_REQ);

    if (req_type == SNS_GENERIC_SAMPLE || req_type == SNS_ANDROID_MUX_SAMPLE)
        req_msg->set_msg_id(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);

    sns_direct_channel_stream_id* stream_id_msg = req_msg->mutable_stream_id();
    if (nullptr == stream_id_msg)
        return;

    sns_std_suid *suid = stream_id_msg->mutable_suid();
    if (nullptr == suid)
        return;

    if (req_type == SNS_GENERIC_SUID) {
        suid->set_suid_low(12370169555311111083ull);
        suid->set_suid_high(12370169555311111083ull);
    }

    if (req_type == SNS_GENERIC_ATTRIBUTES || req_type == SNS_GENERIC_SAMPLE || req_type == SNS_ANDROID_MUX_SAMPLE) {
        if (sensor.sensor_type == "accel") {
            suid->set_suid_low(suid_info_list_.at(0).low);
            suid->set_suid_high(suid_info_list_.at(0).high);
        } else if (sensor.sensor_type == "gyro") {
            suid->set_suid_low(suid_info_list_.at(1).low);
            suid->set_suid_high(suid_info_list_.at(1).high);
        }
    }

    if (req_type == SNS_ANDROID_MUX_SAMPLE) {
        sns_direct_channel_set_client_req_structured_mux_channel_stream_attributes* attributes_msg = req_msg->mutable_attributes();
        if (nullptr == attributes_msg)
            return;

        attributes_msg->set_sensor_handle(0);

        if (sensor.sensor_type == "accel") {
            attributes_msg->set_sensor_type(SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
        } else if (sensor.sensor_type == "gyro") {
            attributes_msg->set_sensor_type(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
        }
    }

    sns_std_request* std_req_msg = req_msg->mutable_request();
    if (nullptr == std_req_msg)
        return;

    stream_id_msg->set_calibrated(false);
    stream_id_msg->set_resampled(false);

    std::string payload = "";
    if (req_type == SNS_GENERIC_SUID) {
        sns_suid_req reg;
        reg.set_data_type(sensor.sensor_type);
        reg.SerializeToString(&payload);
    }
    if (req_type == SNS_GENERIC_ATTRIBUTES) {
        /*This is something like on change sensor. It doesn't have any message
            and corresponding payload. So this should be empty*/
    }
    if (req_type == SNS_GENERIC_SAMPLE || req_type == SNS_ANDROID_MUX_SAMPLE) {
        sns_std_sensor_config std_req;
        std_req.set_sample_rate(float(adjusted_sample_rate));
        std_req.SerializeToString(&payload);
    }
    std_req_msg->set_payload(payload);

    std::string pb_encoded_direct_channel_config_msg;
    config_msg.SerializeToString(&pb_encoded_direct_channel_config_msg);

    int ret =sns_direct_channel_config(fastRPC_remote_handle, channel_handle, (const unsigned char*)pb_encoded_direct_channel_config_msg.c_str(), pb_encoded_direct_channel_config_msg.size());
    if (0 != ret) {
        std::cout <<"sensor_direct_channel send_request failed! " << std::endl;
    }

}

void stop_sensor_streaming(sensors sensor) {
    std::string pb_encoded_direct_channel_config_msg;
    sns_direct_channel_config_msg config_msg;
    int channel_handle = -1;
    char* sensor_buff_ptr = nullptr;
    sns_direct_channel_remove_client_req *rmv_req = config_msg.mutable_remove_client_req();
    if (NULL == rmv_req)
        return;

    sns_direct_channel_stream_id *stream_id_msg = rmv_req->mutable_stream_id();
    if (NULL == stream_id_msg)
        return;

    sns_std_suid *suid = stream_id_msg->mutable_suid();
    if (NULL == suid)
        return;
    if (sensor.sensor_type == "accel") {
        suid->set_suid_low(suid_info_list_.at(0).low);
        suid->set_suid_high(suid_info_list_.at(0).high);
        channel_handle = channel_handle_.at(0);
        sensor_buff_ptr = sensor_buff_ptr_.at(0);
    } else if (sensor.sensor_type == "gyro") {
        suid->set_suid_low(suid_info_list_.at(1).low);
        suid->set_suid_high(suid_info_list_.at(1).high);
        channel_handle = channel_handle_.at(1);
        sensor_buff_ptr = sensor_buff_ptr_.at(1);
    }

    stream_id_msg->set_calibrated(false);
    stream_id_msg->set_resampled(false);

    config_msg.SerializeToString(&pb_encoded_direct_channel_config_msg);
    int ret = sns_direct_channel_config(fastRPC_remote_handle, channel_handle, (const unsigned char*)pb_encoded_direct_channel_config_msg.c_str(), pb_encoded_direct_channel_config_msg.size());
    if (0 == ret) {
        std::cout << "stop_streaming success" << std::endl;
    } else {
        std::cout << "stop_streaming failed" << std::endl;
    }
    delete_direct_channel(channel_handle, sensor_buff_ptr);
}

void read_generic_data_event(sns_request_type event_type, char* sensor_buff_ptr) {
    char* temp = sensor_buff_ptr;
    char *end_ptr = sensor_buff_ptr + DIRECT_CHANNEL_SHARED_MEMORY_SIZE;
    while(1) {
        char* pckt_header_start_ptr = temp;
        char *pckt_header_end_ptr = pckt_header_start_ptr + sizeof(sns_sensor_event);

        if (pckt_header_end_ptr > end_ptr) {
            temp = sensor_buff_ptr;
            continue;
        } else {
            sns_sensor_event event = *reinterpret_cast<const sns_sensor_event *>(temp);
            if (0 == event.timestamp) {
                usleep(1000000);
            } else {
                char *pckt_payload_start_ptr = pckt_header_end_ptr ;
                char *pckt_payload_end_ptr = pckt_header_end_ptr + event.event_len;

                if (pckt_payload_end_ptr > end_ptr) {
                    pckt_payload_start_ptr = sensor_buff_ptr;
                    pckt_payload_end_ptr = pckt_payload_start_ptr + event.event_len;
                }
                /*SUID Event decoding*/
                if (event_type == SNS_GENERIC_SUID) {
                    if (event.message_id == SNS_SUID_MSGID_SNS_SUID_EVENT) {
                        sns_suid_event suid_event_data;
                        suid_event_data.ParseFromArray(pckt_payload_start_ptr , event.event_len);
                        std::string current_data_type = suid_event_data.data_type();
                        int suid_count = suid_event_data.suid_size();
                        for (int i =0 ; i < suid_count ; i++) {
                            suid_info suid;
                            suid.low = suid_event_data.suid(i).suid_low();
                            suid.high = suid_event_data.suid(i).suid_high();
                            suid_info_list_.push_back(suid);
                        }
                        break;
                    }
                }

                /*Attribute Event decoding*/
                if (event_type == SNS_GENERIC_ATTRIBUTES) {
                    if (event.message_id == SNS_STD_MSGID_SNS_STD_ATTR_EVENT) {
                        sns_std_attr_event attr_event_data;
                        attr_event_data.ParseFromArray(pckt_payload_start_ptr, event.event_len);
                        int attr_count = attr_event_data.attributes_size();
                        for (int i = 0 ;i < attr_count ; i++){
                            sns_std_attr attr = attr_event_data.attributes(i);
                            int current_attrib_id = attr.attr_id();
                            sns_std_attr_value attr_value = attr.value();
                            int attr_value_count = attr_value.values_size();
                                for (int i = 0; i < attr_value_count ; i ++) {
                                    sns_std_attr_value_data val = attr_value.values(i);
                                    if (val.has_flt()) {
                                        if (current_attrib_id == SNS_STD_SENSOR_ATTRID_RATES) {
                                            supported_sample_rate_.push_back((int)val.flt());
                                        }
                                    }
                                }
                        }
                        break;
                    }
                }

                temp = pckt_payload_end_ptr;
                if (temp >= end_ptr) {
                    temp = sensor_buff_ptr;
                }
            }
        }
    }
}


void send_fd_to_client(int socket_fd, int fd_number) {
    if (0 == (int)sensors_fd.size()) {
        std::cout << "there is no sensor fd!" << std::endl;
        return;
    }

    msghdr msg;
    cmsghdr *cm;
    int fds[fd_number];
    msg.msg_name = nullptr;
    msg.msg_namelen = 0;
    char buf[CMSG_SPACE(fd_number * sizeof(socket_fd))];

    char tmp[0];
    iovec iov[1];
    iov[0].iov_base = tmp;
    iov[0].iov_len = 1;
    msg.msg_iov = iov;
    msg.msg_iovlen = 1;
    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    cm = CMSG_FIRSTHDR(&msg);
    cm->cmsg_level = SOL_SOCKET;
    cm->cmsg_type = SCM_RIGHTS;
    cm->cmsg_len = CMSG_LEN(fd_number * sizeof(socket_fd));

    // fd copy
    for (int i = 0; i < fd_number; i++){
        fds[i] = sensors_fd[i];
    }

    memcpy((int*)CMSG_DATA(cm), fds, fd_number * sizeof(int));
    sendmsg(socket_fd, &msg, 0);
}

void get_suid(sensors sensor) {
    char* sensor_buff_ptr = nullptr;
    int channel_handle = create_direct_channel(true, sensor_buff_ptr);
    send_config_request(channel_handle, SNS_GENERIC_SUID, sensor);
    read_generic_data_event(SNS_GENERIC_SUID, sensor_buff_ptr);
    delete_direct_channel(channel_handle, sensor_buff_ptr);
}

void get_attributes(sensors sensor) {
    char* sensor_buff_ptr = nullptr;
    int channel_handle = create_direct_channel(true, sensor_buff_ptr);
    send_config_request(channel_handle, SNS_GENERIC_ATTRIBUTES, sensor);
    read_generic_data_event(SNS_GENERIC_ATTRIBUTES, sensor_buff_ptr);
    delete_direct_channel(channel_handle, sensor_buff_ptr);
}

void set_ts_offset(int channel_handle) {
    int64_t ts_offset;
    sensors_timeutil& timeutil = sensors_timeutil::get_instance();
    timeutil.recalculate_offset(true);
    ts_offset = timeutil.getElapsedRealtimeNanoOffset();
    std::cout << "sensor-service ts_offset " << ts_offset << std::endl;

    sns_direct_channel_config_msg config_msg;
    sns_direct_channel_set_ts_offset *offset_msg = config_msg.mutable_set_ts_offset();
    if (nullptr == offset_msg)
        return;
    offset_msg->set_ts_offset(ts_offset);
    std::string pb_encoded_direct_channel_config_msg;
    config_msg.SerializeToString(&pb_encoded_direct_channel_config_msg);
    int ret = sns_direct_channel_config(fastRPC_remote_handle, channel_handle, (const unsigned char*)pb_encoded_direct_channel_config_msg.c_str(), pb_encoded_direct_channel_config_msg.size());
    if (0 != ret) {
        std::cout << "sensor_direct_channel set_ts_offset failed " << std::endl;
    }
}

void start_sensor_streaming(sensors sensor, int adjusted_sample_rate) {
    char* sensor_buff_ptr = nullptr;
    int channel_handle = create_direct_channel(false, sensor_buff_ptr, true, sensor.sensor_type);
    channel_handle_.push_back(channel_handle);
    sensor_buff_ptr_.push_back(sensor_buff_ptr);
    set_ts_offset(channel_handle);
    send_config_request(channel_handle, SNS_ANDROID_MUX_SAMPLE, sensor, adjusted_sample_rate);
}

int caluclate_adjusted_sample_rate(int resample_type, int sample_rate_hz, std::vector<int> supported_sample_rate) {
    int asked = sample_rate_hz;
    int adjusted = -1;
    for (int i =0 ; i < (int)supported_sample_rate.size(); i ++) {
        if (supported_sample_rate.at(i) < asked ) {
            continue;
        } else {
            adjusted = supported_sample_rate.at(i);
            break;
        }
    }
    if (adjusted == -1) {
        adjusted = asked;
    }
    std::cout << "Asked for sample_rate "<< asked << " Adjusted sample_rate "
                << adjusted << std::endl;
    return adjusted;
}

void get_fd_of_sensor_data(sensors sensor) {
    adjusted_sample_rate = caluclate_adjusted_sample_rate(sensor.resampled, sensor.sample_rate, supported_sample_rate_);
    start_sensor_streaming(sensor, adjusted_sample_rate);
}

int get_sensors_info(std::string sensors_info_path) {
    std::ifstream fin;
    Json::Reader reader;
    Json::Value root;
    Json::Value sensors_info;

    fin.open(sensors_info_path, std::ios::in);
    if (!fin.is_open()) {
        std::cout << "There is no sensors_info conf, use default conf." << std::endl;
        fin.open(CONFIG_FILE_DEFAULT_PATH, std::ios::in);
        if (!fin.is_open()) {
            std::cout << "No default conf file found!" << std::endl;
            return -1;
        } else {
            if (reader.parse(fin, root)) {
                if (root["version"] == SensorsConfVer) {
                    sensors_info = root["sensors"];
                    int sensors_size = sensors_info.size();
                    for (int i = 0; i < sensors_size; i++) {
                    sensors sensor;
                    sensor.sensor_type = sensors_info[i][SENSOR_TYPE].asString();
                    sensor.calibrated = 0;
                    sensor.resampled = 0;
                    sensor.sample_rate = sensors_info[i][SAMPLE_RATE].asInt();
                    original_sample_rate = sensor.sample_rate;
                    sensor.channel_type = 0;
                    sensors_info_vec_.emplace_back(sensor);
                    }
                }
            } else {
                std::cout << "Can't parse sensors_info_conf.default file." << std::endl;
                return -1;
            }
        }
    } else {
        if (reader.parse(fin, root)) {
            if (root["version"] == SensorsConfVer) {
                sensors_info = root["sensors"];
                int sensors_size = sensors_info.size();
                if (sensors_size != CURRENT_SUPPORT_SENSOR_NUM) {
                    std::cout << "Now support accel/gyro, must config both." << std::endl;
                    return -1;
                }
                for (int i = 0; i < sensors_size; i++) {
                    sensors sensor;
                    sensor.sensor_type = sensors_info[i][SENSOR_TYPE].asString();
                    if (sensor.sensor_type != SENSOR_ACCEL && sensor.sensor_type != SENSOR_GYRO) {
                        std::cout << "sensor_type error! Now only support accel/gyro." << std::endl;
                        return -1;
                    }
                    sensor.calibrated = 0;
                    sensor.resampled = 0;
                    sensor.sample_rate = sensors_info[i][SAMPLE_RATE].asInt();
                    original_sample_rate = sensor.sample_rate;
                    sensor.channel_type = 0;
                    for (auto &itor : sensors_info_vec_) {
                        if (itor.sensor_type == sensor.sensor_type) {
                            std::cout << "sensor "<< itor.sensor_type << " already conf! Plz don't duplicate conf." << std::endl;
                            return -1;
                        }
                    }
                    sensors_info_vec_.emplace_back(sensor);
                }
            } else {
                std::cout << "sensors_info.conf version not correct! Current version:" << SensorsConfVer << std::endl;
                return -1;
            }
        } else {
            std::cout << "Can't parse sensors_info conf file." << std::endl;
            return -1;
        }
    }

    return 0;
}


void print_sensors_info(std::vector<sensors> &sensors_info) {
    for (auto &sensor : sensors_info) {
        std::cout << sensor.sensor_type << std::endl;
        std::cout << sensor.calibrated << std::endl;
        std::cout << sensor.resampled << std::endl;
        std::cout << sensor.sample_rate << std::endl;
        std::cout << sensor.channel_type << std::endl;
        std::cout << std::endl;
    }
}

void get_support_sample_rate(sensors sensor) {
    get_suid(sensor);
    get_attributes(sensor);
}

int verify_sensors_info(sensors sensor) {
    int support_sample_rate_size = supported_sample_rate_.size();
    if (sensor.sample_rate < supported_sample_rate_[0] || sensor.sample_rate > supported_sample_rate_[support_sample_rate_size - 1]) {
        std::cout << "sample_rate conf error out of range!" << std::endl;
        return -1;
    }
    return 0;
}

void wait_for_connect() {
    int service_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un server_addr;
    server_addr.sun_family = AF_UNIX;

    char server_socket_name[MAX_PATH_LEN] = SOCKET_PATH;
    unlink(server_socket_name);
    strlcpy(server_addr.sun_path, server_socket_name, MAX_STRING_LEN);

    if (bind(service_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cout << "bind failed" << std::endl;
        exit(1);
    }

    if (listen(service_fd, MAX_SOCK_CLIENT) < 0) {
        std::cout << "Listen failed" << std::endl;
        exit(1);
    }

    struct sockaddr_un client_addr;
    socklen_t len = sizeof(client_addr);

    while(1) {
        int client_fd = accept(service_fd, (struct sockaddr *)&client_addr, &len);
        std::cout << "service recv from client:" << client_fd << std::endl;
        mtx.lock();
        if (num == 0) {
            for (int i = 0; i < CURRENT_SUPPORT_SENSOR_NUM; i++) {
                get_fd_of_sensor_data(sensors_info_vec_[i]);
            }
        }
        num++;
        mtx.unlock();
        int msg_socket[2] = {original_sample_rate, adjusted_sample_rate};
        int send_len = send(client_fd, &msg_socket, sizeof(msg_socket), 0);
        if (send_len < 0) {
            std::cout << "sensor-service send sample_rate failed" << std::endl;
        }

        send_fd_to_client(client_fd, CURRENT_SUPPORT_SENSOR_NUM);

        auto func = [&client_fd, &num, &mtx]() -> void
            {
                int msg[1] = {-1};
                int len = recv(client_fd, &msg, sizeof(msg),0);
                std::cout << "service recv from msg: " << msg[0] << " len: " << len << std::endl;
                if (msg[0] == STOP || len <= 0) {
                    mtx.lock();
                    num--;
                    if (num == 0) {
                        for (int i = 0; i < (int)sensors_info_vec_.size(); i++) {
                            stop_sensor_streaming(sensors_info_vec_[i]);
                        }
                        sensors_fd.clear();
                    }
                    mtx.unlock();
                    close(client_fd);
                }
            };
        auto thread = std::make_shared<std::thread>(func);
        thread->detach();
    }

    close(service_fd);
}

int main(int argc, char *argv[])
{
    int sensro_info_ret = get_sensors_info(CONFIG_FILE_PATH);
    if (sensro_info_ret < 0) {
        std::cout << "get_sensors_info fail." << std::endl;
        return -1;
    }
    print_sensors_info(sensors_info_vec_);

    int ret = fastRPC_remote_handle_init();
    if (ret < 0) {
        std::cout << "fastRPC_remote_handle_init fail." << std::endl;
        return -1;
    }

    for (int i = 0; i < (int)sensors_info_vec_.size(); i++) {
        get_support_sample_rate(sensors_info_vec_[i]);
    }

    std::cout << "Now support sample_rate" << std::endl;
    for (int i = 0; i < supported_sample_rate_.size(); i++) {
        std::cout << supported_sample_rate_[i] << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < (int)sensors_info_vec_.size(); i++) {
        int sample_rate_valid = verify_sensors_info(sensors_info_vec_[i]);
        if (sample_rate_valid < 0) {
            std::cout << "sample_rate conf error!" << std::endl;
            return -1;
        }
    }

    wait_for_connect();
    return 0;
}

