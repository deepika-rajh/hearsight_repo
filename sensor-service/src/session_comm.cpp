/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "sensor_service/comm/session_comm.hpp"

#include <iostream>

using namespace std;
using namespace ::com::quic::sensinghub::session::V1_0;

bool SessionComm::init()
{
  if (init_ == true) {
    cout << "ISession has inited" << endl;
    return true;
  }
  factory_ = std::make_shared<sessionFactory>();

  init_ = true;
  sampling_frequencies_list_.resize(SENSOR_SIZE);
  suid_list_.resize(SENSOR_SIZE);
  session_list_.resize(SENSOR_SIZE, nullptr);
  frequency_adapt_list_.resize(SENSOR_SIZE);
  buffer_list_.resize(SENSOR_SIZE, nullptr);
  for (int i = 0; i < SENSOR_SIZE; i++) {
    reserve_count_[i] = 0;
  }
  return true;
}

bool SessionComm::get_suid(const SensorType type)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return false;
  }
  if (suid_list_[index].low != 0 && suid_list_[index].high != 0) {
    return true;
  }
  auto sensor_type = convert_sensor_type_to_string(type);

  ISession::eventCallBack suidEvent = [&, index](
                                          const uint8_t * data, size_t size, uint64_t timeStamp) {
    event_process(data, size, timeStamp, index);
    unique_lock<mutex> lock(suid_mtx_);
    suid_cv_.notify_one();
  };

  ISession::respCallBack suidResp = [](uint32_t resp_value, uint64_t client_connect_id) {
    cout << "SUID discovery response received." << endl;
  };

  ISession::errorCallBack suidError = [](ISession::error errorValue) {};

  suid uid;
  sns_suid_sensor suid_sensor;
  uid.low = (uint64_t)suid_sensor.suid_low();
  uid.high = (uint64_t)suid_sensor.suid_high();

  unique_ptr<ISession> suidSession = unique_ptr<ISession>(factory_->getSession());
  if (nullptr == suidSession) {
    cout << "failed to create session for suid discovery" << endl;
    return false;
  }

  int ret = suidSession->open();
  if (-1 == ret) {
    cout << "failed to open ISession for suid discovery" << endl;
    return false;
  }

  ret = suidSession->setCallBacks(uid, suidResp, suidError, suidEvent);
  if (0 != ret)
    cout << "all callbacks are null, no need to register it" << endl;

  string pb_req_encoded = "";

  sns_suid_req pb_suid_req;
  pb_suid_req.set_data_type(sensor_type);
  pb_suid_req.set_register_updates(true);
  pb_suid_req.set_default_only(true);
  pb_suid_req.SerializeToString(&pb_req_encoded);

  sns_client_request_msg pb_req_msg;
  pb_req_msg.set_msg_id(SNS_SUID_MSGID_SNS_SUID_REQ);
  pb_req_msg.mutable_request()->set_payload(pb_req_encoded);
  pb_req_msg.mutable_suid()->set_suid_high(uid.high);
  pb_req_msg.mutable_suid()->set_suid_low(uid.low);
  pb_req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
  pb_req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);

  string pb_req_msg_encoded;
  pb_req_msg.SerializeToString(&pb_req_msg_encoded);

  /* send proto encoded message to sensing-hub using the suidSession */
  ret = suidSession->sendRequest(uid, pb_req_msg_encoded);
  if (0 != ret) {
    cout << "Error in sending suid discovery request" << endl;
    return false;
  }

  /* wait until all suids are received */
  unique_lock<mutex> eventLock(suid_mtx_);
  suid_cv_.wait(eventLock);

  /* close the session once suids are received */
  suidSession->close();

  return true;
}

void SessionComm::event_process(const uint8_t * data,
    size_t size,
    uint64_t timeStamp,
    int index,
    bool process_data)
{
  sns_client_event_msg pb_event_msg;
  /* Parse the pb encoded event buffer */
  pb_event_msg.ParseFromArray(data, size);

  for (int i = 0; i < pb_event_msg.events_size(); i++) {
    auto & pb_event = pb_event_msg.events(i);

    /* parse attribute event packet */
    if (pb_event.msg_id() == SNS_STD_MSGID_SNS_STD_ATTR_EVENT && !process_data) {
      sns_std_attr_event pb_attr_event;
      pb_attr_event.ParseFromString(pb_event.payload());

      for (int i = 0; i < pb_attr_event.attributes_size(); i++) {
        sns_std_attr attr = pb_attr_event.attributes(i);
        // std::cout<<"attr.attr_id "<<attr.attr_id()<<", SNS_STD_SENSOR_ATTRID_RATES = "
        // <<SNS_STD_SENSOR_ATTRID_RATES<<std::endl;
        if (attr.attr_id() == SNS_STD_SENSOR_ATTRID_RATES) {  // SNS_STD_SENSOR_ATTRID_RATES
          sns_std_attr_value attr_value = attr.value();
          int attr_value_count = attr_value.values_size();
          for (int i = 0; i < attr_value_count; i++) {
            sns_std_attr_value_data val = attr_value.values(i);
            // printf("flt: %f\t" , val.flt());
            sampling_frequencies_list_[index].push_back((int)val.flt());
          }
          break;
        }
      }
    } else if (pb_event.msg_id() == SNS_SUID_MSGID_SNS_SUID_EVENT && !process_data) {
      sns_suid_event pb_suid_event;
      pb_suid_event.ParseFromString(pb_event.payload());
      const string & datatype = pb_suid_event.data_type();
      std::cout << "Received SUIDs for " << datatype.c_str()
                << ", number of suids received = " << pb_suid_event.suid_size() << std::endl;

      /* create a list of all suids found for this sensorType */
      for (int j = 0; j < pb_suid_event.suid_size(); j++) {
        std::cout << "SUID received - suid_low= " << pb_suid_event.suid(j).suid_low()
                  << ", suid_high= " << pb_suid_event.suid(j).suid_high() << std::endl;
        suid_list_[index] =
            suid(pb_suid_event.suid(j).suid_low(), pb_suid_event.suid(j).suid_high());
      }
    } else if (pb_event.msg_id() == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT) {
      // printf("\nReceived Samples:\t");
      sns_std_sensor_event pb_stream_event;
      pb_stream_event.ParseFromString(pb_event.payload());
      // std::cout << "timestamp: "<<timeStamp <<" pb stream size: "<<pb_stream_event.data_size()
      // <<std::endl;
      auto event = buffer_list_[index]->get_writable_item();
      if (event == nullptr) {
        continue;
      }
      event->timestamp = timeStamp;
      if (index == 0) {
        event->acceleration.x = pb_stream_event.data(0);
        event->acceleration.y = pb_stream_event.data(1);
        event->acceleration.z = pb_stream_event.data(2);
      } else if (index == 1) {
        event->gyro.x = pb_stream_event.data(0);
        event->gyro.y = pb_stream_event.data(1);
        event->gyro.z = pb_stream_event.data(2);
      }
      {
        unique_lock<mutex> lock(reserve_mtx_[index]);
        event->reserved0 = reserve_count_[index];
        reserve_count_[index]++;
      }
    }

    /* parse cal event packet */
    else if (pb_event.msg_id() == SNS_CAL_MSGID_SNS_CAL_EVENT) {
      printf("\nCal event packet received");
    }

    /* parse sensor re-configuration event packet */
    else if (pb_event.msg_id() == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT) {
      printf("\nReceived re-configuration event");
    } else
      printf("\ninvalid event msg_id = %d", pb_event.msg_id());
  }
}

bool SessionComm::get_attributes(const SensorType type)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return false;
  }
  if (suid_list_[index].low == 0 && suid_list_[index].high == 0) {
    get_suid(type);
  }
  auto sensor_type = convert_sensor_type_to_string(type);

  ISession::eventCallBack attributeEvent = [&, index](const uint8_t * data, size_t size,
                                               uint64_t timeStamp) {
    event_process(data, size, timeStamp, index);
    unique_lock<mutex> lock(attr_mtx_);
    attr_cv_.notify_one();
  };

  ISession::respCallBack attributeResp = [](uint32_t resp_value, uint64_t client_connect_id) {
    cout << "\nAttribute query response received.";
  };

  ISession::errorCallBack attributeError = [](ISession::error errorValue) {
    /* User may add their own error handling mechanism incase any error is received */
  };

  unique_ptr<ISession> attributeSession = unique_ptr<ISession>(factory_->getSession());
  if (nullptr == attributeSession) {
    printf("failed to create session for attribute query");
    return false;
  }

  /* open the attributeSession session */
  int ret = attributeSession->open();
  if (-1 == ret) {
    printf("failed to open ISession for attribute query");
    return false;
  }
  auto uid = suid_list_[index];
  /* set callbacks for the session for 'uid' */
  ret = attributeSession->setCallBacks(uid, attributeResp, attributeError, attributeEvent);
  if (0 != ret)
    printf("all callbacks are null, no need to register it");

  /* create pb-encoded config request message to be sent for attribute query */
  sns_client_request_msg pb_req_msg;
  pb_req_msg.set_msg_id(SNS_STD_MSGID_SNS_STD_ATTR_REQ);
  pb_req_msg.mutable_request()->clear_payload();
  pb_req_msg.mutable_suid()->set_suid_high(uid.high);
  pb_req_msg.mutable_suid()->set_suid_low(uid.low);
  pb_req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
  pb_req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);

  string pb_req_msg_encoded;
  pb_req_msg.SerializeToString(&pb_req_msg_encoded);

  /* send proto encoded message to sensing-hub using the attributeSession */
  ret = attributeSession->sendRequest(uid, pb_req_msg_encoded);
  if (0 != ret) {
    printf("Error in sending attribute query request");
    return false;
  }

  /* wait until all requested attributes are received */
  unique_lock<mutex> lock(attr_mtx_);
  attr_cv_.wait(lock);

  /* close the session once all attributes are received */
  attributeSession->close();
  printf("\nAttributes for all suids received\n");
  return true;
}

bool SessionComm::start_sensor_sampling(const SensorType type,
    const int request_frequency,
    int & adjust_frequency)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return false;
  }

  if (sampling_frequencies_list_[index].size() == 0) {
    get_attributes(type);
  }

  for (auto support_frequency : sampling_frequencies_list_[index]) {
    if (support_frequency >= request_frequency) {
      adjust_frequency = support_frequency;
      break;
    }
  }

  if (session_list_[index] != nullptr) {
    return true;
  }

  if (buffer_list_[index] == nullptr) {
    buffer_list_[index] = std::make_shared<CycleBuffer>(
        sizeof(sensors_event_t), 1000, convert_sensor_type_to_string(type));
  }

  ISession * streamingSession = factory_->getSession();
  if (nullptr == streamingSession) {
    printf("failed to create streaming session");
    return false;
  }

  int ret = streamingSession->open();
  if (-1 == ret) {
    printf("failed to open ISession for attribute query");
    return false;
  }

  session_list_[index] = streamingSession;

  ISession::eventCallBack dataEvent = [&, index](
                                          const uint8_t * data, size_t size, uint64_t timeStamp) {
    event_process(data, size, timeStamp, index, true);
  };

  ISession::respCallBack dataResp = [](uint32_t resp_value, uint64_t client_connect_id) {
    cout << "\nData request response received.";
  };

  ISession::errorCallBack dataError = [](ISession::error errorValue) {
    /* User may add their own error handling mechanism incase any error is received */
  };

  cout << "index: " << index << " adjust_frequency: " << adjust_frequency << std::endl;

  static const uint64_t USEC_PER_SEC = 1000000ull;
  int batchPeriodMicroSec = USEC_PER_SEC / adjust_frequency;

  auto uid = suid_list_[index];
  /* set callbacks for the session for 'uid' */
  std::cout << "sending request suid_low= " << uid.low << ", suid_high= " << uid.high << std::endl;
  ret = streamingSession->setCallBacks(uid, dataResp, dataError, dataEvent);
  if (0 != ret)
    printf("all callbacks are null, no need to register it");

  /* create pb-encoded config request message to be sent for streaming request */
  string pb_req_encoded = "";

  sns_std_sensor_config pb_stream_cfg;
  pb_stream_cfg.set_sample_rate(adjust_frequency);
  pb_stream_cfg.SerializeToString(&pb_req_encoded);

  sns_client_request_msg pb_req_msg;
  pb_req_msg.mutable_request()->mutable_batching()->set_batch_period(batchPeriodMicroSec);
  pb_req_msg.set_msg_id(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
  pb_req_msg.mutable_request()->set_payload(pb_req_encoded);
  pb_req_msg.mutable_suid()->set_suid_high(uid.high);
  pb_req_msg.mutable_suid()->set_suid_low(uid.low);
  pb_req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
  pb_req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);

  string pb_req_msg_encoded;
  pb_req_msg.SerializeToString(&pb_req_msg_encoded);

  /* send proto encoded message to sensing-hub using the streamingSession */
  ret = streamingSession->sendRequest(uid, pb_req_msg_encoded);
  if (0 != ret) {
    printf("Error in sending streaming request");
    return false;
  }

  return true;
}

void SessionComm::stop_sensor_sampling(const SensorType type)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return;
  }

  if (session_list_[index] == nullptr) {
    return;
  }

  auto uid = suid_list_[index];
  sns_client_request_msg pb_req_msg;

  pb_req_msg.set_msg_id(SNS_CLIENT_MSGID_SNS_CLIENT_DISABLE_REQ);
  pb_req_msg.mutable_suid()->set_suid_high(uid.high);
  pb_req_msg.mutable_suid()->set_suid_low(uid.low);
  pb_req_msg.mutable_susp_config()->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
  pb_req_msg.mutable_susp_config()->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
  pb_req_msg.mutable_request()->mutable_batching()->set_flush_period(UINT32_MAX);
  pb_req_msg.mutable_request()->mutable_batching()->set_batch_period(0);
  pb_req_msg.mutable_request()->clear_payload();

  string pb_req_msg_encoded;
  pb_req_msg.SerializeToString(&pb_req_msg_encoded);

  /* send disable request to sensing-hub */
  int ret = session_list_[index]->sendRequest(uid, pb_req_msg_encoded);
  if (0 != ret) {
    printf("Error in sending disable request");
    return;
  }
  session_list_[index]->close();
  delete session_list_[index];
  session_list_[index] = nullptr;
  buffer_list_[index] = nullptr;
  {
    unique_lock<mutex> lock(reserve_mtx_[index]);
    reserve_count_[index] = 0;
  }
}

bool SessionComm::get_sensor_available_sampling_frequency(const SensorType type,
    std::vector<int> & frequencies)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return false;
  }
  auto it = sampling_frequencies_list_[index];
  if (it.size() != 0) {
    frequencies = it;
    return true;
  }

  // read suid info
  bool ret = get_suid(type);

  // read available frequency list
  ret = get_attributes(type);
  frequencies = sampling_frequencies_list_[index];

  return ret;
}

bool SessionComm::get_sensor_fd(const SensorType type, int & fd)
{
  auto index = convert_sensor_type_to_index(type);
  if (index < 0) {
    return false;
  }

  if (session_list_[index] == nullptr || buffer_list_[index]->get_fd() == -1) {
    return false;
  }

  fd = buffer_list_[index]->get_fd();
  return true;
}
