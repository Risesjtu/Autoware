#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "visualization.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using visualization::Frame;
using visualization::Visualizer;

class Sender
{
private:
    /* data */
public:
    std::string address_;
    std::shared_ptr<Channel> channel_ptr_;
    std::unique_ptr<Visualizer::Stub> stub_ptr_; 
    Sender();
    ~Sender(){};
    void send(const visualization::Frame &msg);
};