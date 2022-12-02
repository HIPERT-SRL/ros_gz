// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERVICE_FACTORY_HPP_
#define SERVICE_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include "ros_gz_bridge/convert_decl.hpp"

#include "service_factory_interface.hpp"

namespace ros_gz_bridge {

template <typename RosResT> bool send_response_on_error(RosResT &ros_response);

static std::unordered_map<std::string,
                          std::pair<std::shared_ptr<gz::transport::Node>,
                                    std::shared_ptr<rclcpp::ServiceBase>>>
    sServices;

template <typename RosServiceT, typename GzRequestT, typename GzReplyT>
class ServiceFactory : public ServiceFactoryInterface {
public:
  ServiceFactory(const std::string &ros_type_name,
                 const std::string &gz_req_type_name,
                 const std::string &gz_rep_type_name)
      : ros_type_name_(ros_type_name), gz_req_type_name_(gz_req_type_name),
        gz_rep_type_name_(gz_rep_type_name) {}

  rclcpp::ServiceBase::SharedPtr
  create_ros_service(rclcpp::Node::SharedPtr ros_node,
                     std::shared_ptr<gz::transport::Node> gz_node,
                     const std::string &service_name) override {

    std::function<void(
        const std::shared_ptr<rmw_request_id_t> reqid,
        const std::shared_ptr<typename RosServiceT::Request> ros_req,
        std::shared_ptr<typename RosServiceT::Response> ros_res)>
        callbackFunction =
            std::bind(&ServiceFactory::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, service_name);
    std::shared_ptr<rclcpp::Service<RosServiceT>> srv_handle =
        ros_node->create_service<RosServiceT>(service_name, callbackFunction);

    std::cout << "Create " << service_name << std::endl;
    sServices[service_name] = std::make_pair(std::move(gz_node), srv_handle);

    std::cout << sServices[service_name].first.get() << " "
              << sServices[service_name].second.get() << std::endl;

    std::cout << "size: " << sServices.size() << std::endl;

    return srv_handle;
  }

private:
  void service_callback(std::shared_ptr<rmw_request_id_t> reqid,
                        std::shared_ptr<typename RosServiceT::Request> ros_req,
                        std::shared_ptr<typename RosServiceT::Response> ros_res,
                        const std::string &service_name) {

    std::function<void(const GzReplyT &, bool)> callback =
        [this, ros_res, service_name, reqid](const GzReplyT &reply,
                                             const bool result) {
          rclcpp::Service<RosServiceT> *srv_handle =
              static_cast<rclcpp::Service<RosServiceT> *>(
                  sServices[service_name].second.get());
          if (!result) {
            if (send_response_on_error(*ros_res)) {
              srv_handle->send_response(*reqid, *ros_res);
            }
          }
          convert_gz_to_ros(reply, *ros_res);
          srv_handle->send_response(*reqid, *ros_res);
        };

    GzRequestT gz_req;
    convert_ros_to_gz(*ros_req, gz_req);

    sServices[service_name].first->Request(service_name, gz_req, callback);

    reqid->sequence_number--;
    std::cout << reqid->sequence_number << std::endl;
  }

  std::string ros_type_name_;
  std::string gz_req_type_name_;
  std::string gz_rep_type_name_;
};

} // namespace ros_gz_bridge

#endif // SERVICE_FACTORY_HPP_
